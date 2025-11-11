using Microsoft.FlightSimulator.SimConnect;
using System.Runtime.InteropServices;

namespace SimpleUartReceiver;

/// <summary>
/// Manages SimConnect connection and autopilot interactions with MSFS2024
/// </summary>
public class SimConnectManager : IDisposable
{
    private SimConnect? _simConnect;
    private bool _isConnected;
    private readonly object _lock = new();
    private int _currentHeading;
    private bool _headingValid;
    private int _currentAltitude;
    private bool _altitudeValid;
    private int _currentVerticalSpeed;
    private bool _verticalSpeedValid;

    // Autopilot status tracking
    private bool _apMasterEngaged;
    private bool _apHeadingActive;
    private bool _apAltitudeActive;
    private bool _apVerticalSpeedActive;
    private bool _apStatusValid;

    // SimConnect enum definitions
    private enum DEFINITIONS
    {
        AutopilotData
    }

    private enum REQUESTS
    {
        AutopilotDataRequest
    }

    private enum EVENTS
    {
        SetHeading,
        SetAltitude,
        SetVerticalSpeed
    }

    private enum GROUPS
    {
        Default
    }

    // Data structure for autopilot data
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct AutopilotData
    {
        public double HeadingBug;         // AUTOPILOT HEADING LOCK DIR
        public double AltitudeBug;        // AUTOPILOT ALTITUDE LOCK VAR (feet)
        public double VerticalSpeedBug;   // AUTOPILOT VERTICAL HOLD VAR (feet per minute)
        public int ApMaster;              // AUTOPILOT MASTER (1 = engaged, 0 = disengaged)
        public int ApHeadingLock;         // AUTOPILOT HEADING LOCK (1 = active, 0 = inactive)
        public int ApAltitudeLock;        // AUTOPILOT ALTITUDE LOCK (1 = active, 0 = inactive)
        public int ApVerticalHold;        // AUTOPILOT VERTICAL HOLD (1 = active, 0 = inactive)
    }

    public bool IsConnected
    {
        get { lock (_lock) return _isConnected; }
    }

    public int CurrentHeading
    {
        get { lock (_lock) return _currentHeading; }
    }

    public int CurrentAltitude
    {
        get { lock (_lock) return _currentAltitude; }
    }

    public int CurrentVerticalSpeed
    {
        get { lock (_lock) return _currentVerticalSpeed; }
    }

    public bool IsAutopilotEngaged
    {
        get { lock (_lock) return _apMasterEngaged; }
    }

    public bool IsHeadingModeActive
    {
        get { lock (_lock) return _apHeadingActive; }
    }

    public bool IsAltitudeModeActive
    {
        get { lock (_lock) return _apAltitudeActive; }
    }

    public bool IsVerticalSpeedModeActive
    {
        get { lock (_lock) return _apVerticalSpeedActive; }
    }

    public bool IsAutopilotStatusValid
    {
        get { lock (_lock) return _apStatusValid; }
    }

    /// <summary>
    /// Initialize SimConnect connection to MSFS
    /// </summary>
    public bool Connect()
    {
        lock (_lock)
        {
            if (_isConnected)
                return true;

            try
            {
                _simConnect = new SimConnect("AutopilotPanel", IntPtr.Zero, 0, null, 0);

                // Register event handlers
                _simConnect.OnRecvOpen += OnRecvOpen;
                _simConnect.OnRecvQuit += OnRecvQuit;
                _simConnect.OnRecvException += OnRecvException;
                _simConnect.OnRecvSimobjectData += OnRecvSimobjectData;

                // Define data structure for autopilot data
                _simConnect.AddToDataDefinition(
                    DEFINITIONS.AutopilotData,
                    "AUTOPILOT HEADING LOCK DIR",
                    "degrees",
                    SIMCONNECT_DATATYPE.FLOAT64,
                    0.0f,
                    SimConnect.SIMCONNECT_UNUSED
                );

                _simConnect.AddToDataDefinition(
                    DEFINITIONS.AutopilotData,
                    "AUTOPILOT ALTITUDE LOCK VAR",
                    "feet",
                    SIMCONNECT_DATATYPE.FLOAT64,
                    0.0f,
                    SimConnect.SIMCONNECT_UNUSED
                );

                _simConnect.AddToDataDefinition(
                    DEFINITIONS.AutopilotData,
                    "AUTOPILOT VERTICAL HOLD VAR",
                    "feet per minute",
                    SIMCONNECT_DATATYPE.FLOAT64,
                    0.0f,
                    SimConnect.SIMCONNECT_UNUSED
                );

                _simConnect.AddToDataDefinition(
                    DEFINITIONS.AutopilotData,
                    "AUTOPILOT MASTER",
                    "bool",
                    SIMCONNECT_DATATYPE.INT32,
                    0.0f,
                    SimConnect.SIMCONNECT_UNUSED
                );

                _simConnect.AddToDataDefinition(
                    DEFINITIONS.AutopilotData,
                    "AUTOPILOT HEADING LOCK",
                    "bool",
                    SIMCONNECT_DATATYPE.INT32,
                    0.0f,
                    SimConnect.SIMCONNECT_UNUSED
                );

                _simConnect.AddToDataDefinition(
                    DEFINITIONS.AutopilotData,
                    "AUTOPILOT ALTITUDE LOCK",
                    "bool",
                    SIMCONNECT_DATATYPE.INT32,
                    0.0f,
                    SimConnect.SIMCONNECT_UNUSED
                );

                _simConnect.AddToDataDefinition(
                    DEFINITIONS.AutopilotData,
                    "AUTOPILOT VERTICAL HOLD",
                    "bool",
                    SIMCONNECT_DATATYPE.INT32,
                    0.0f,
                    SimConnect.SIMCONNECT_UNUSED
                );

                _simConnect.RegisterDataDefineStruct<AutopilotData>(DEFINITIONS.AutopilotData);

                // Map events for setting autopilot bugs
                _simConnect.MapClientEventToSimEvent(EVENTS.SetHeading, "HEADING_BUG_SET");
                _simConnect.MapClientEventToSimEvent(EVENTS.SetAltitude, "AP_ALT_VAR_SET_ENGLISH");
                _simConnect.MapClientEventToSimEvent(EVENTS.SetVerticalSpeed, "AP_VS_VAR_SET_ENGLISH");

                // Add events to notification group
                _simConnect.AddClientEventToNotificationGroup(GROUPS.Default, EVENTS.SetHeading, false);
                _simConnect.AddClientEventToNotificationGroup(GROUPS.Default, EVENTS.SetAltitude, false);
                _simConnect.AddClientEventToNotificationGroup(GROUPS.Default, EVENTS.SetVerticalSpeed, false);
                _simConnect.SetNotificationGroupPriority(GROUPS.Default, SimConnect.SIMCONNECT_GROUP_PRIORITY_HIGHEST);

                // Request autopilot data updates
                _simConnect.RequestDataOnSimObject(
                    REQUESTS.AutopilotDataRequest,
                    DEFINITIONS.AutopilotData,
                    SimConnect.SIMCONNECT_OBJECT_ID_USER,
                    SIMCONNECT_PERIOD.SECOND,
                    SIMCONNECT_DATA_REQUEST_FLAG.DEFAULT,
                    0, 0, 0
                );

                Console.WriteLine("[SimConnect] Connection initiated...");
                return true;
            }
            catch (COMException ex)
            {
                Console.WriteLine($"[SimConnect] Failed to connect: {ex.Message}");
                _simConnect = null;
                _isConnected = false;
                return false;
            }
        }
    }

    /// <summary>
    /// Process SimConnect messages (call this periodically)
    /// </summary>
    public void ReceiveMessage()
    {
        lock (_lock)
        {
            try
            {
                _simConnect?.ReceiveMessage();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[SimConnect] Error receiving message: {ex.Message}");
            }
        }
    }

    /// <summary>
    /// Adjust heading bug by delta degrees
    /// </summary>
    /// <param name="delta">Degrees to add/subtract (-180 to +180)</param>
    /// <returns>True if command was sent successfully</returns>
    public bool AdjustHeading(int delta)
    {
        lock (_lock)
        {
            if (!_isConnected)
            {
                Console.WriteLine("[SimConnect] Not connected - discarding HDG command");
                return false;
            }

            if (!_headingValid)
            {
                Console.WriteLine("[SimConnect] Current heading unknown - cannot apply delta");
                return false;
            }

            try
            {
                // Calculate new heading with wraparound
                int newHeading = _currentHeading + delta;
                while (newHeading < 0) newHeading += 360;
                while (newHeading >= 360) newHeading -= 360;

                Console.WriteLine($"[SimConnect] HDG: {_currentHeading}° + {delta:+#;-#;0}° = {newHeading}°");

                // Send heading bug set event
                _simConnect?.TransmitClientEvent(
                    SimConnect.SIMCONNECT_OBJECT_ID_USER,
                    EVENTS.SetHeading,
                    (uint)newHeading,
                    GROUPS.Default,
                    SIMCONNECT_EVENT_FLAG.DEFAULT
                );

                // Update our cached value
                _currentHeading = newHeading;
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[SimConnect] Failed to adjust heading: {ex.Message}");
                return false;
            }
        }
    }

    /// <summary>
    /// Adjust altitude bug by delta feet (typically 100ft increments)
    /// </summary>
    /// <param name="delta">Feet to add/subtract</param>
    /// <returns>True if command was sent successfully</returns>
    public bool AdjustAltitude(int delta)
    {
        lock (_lock)
        {
            if (!_isConnected)
            {
                Console.WriteLine("[SimConnect] Not connected - discarding ALT command");
                return false;
            }

            if (!_altitudeValid)
            {
                Console.WriteLine("[SimConnect] Current altitude unknown - cannot apply delta");
                return false;
            }

            try
            {
                // Calculate new altitude (clamp to reasonable values: 0 to 50,000 feet)
                int newAltitude = _currentAltitude + delta * 100;
                if (newAltitude < 0) newAltitude = 0;
                if (newAltitude > 50000) newAltitude = 50000;

                Console.WriteLine($"[SimConnect] ALT: {_currentAltitude}ft + {delta * 100:+#;-#;0}ft = {newAltitude}ft");

                // Send altitude bug set event
                _simConnect?.TransmitClientEvent(
                    SimConnect.SIMCONNECT_OBJECT_ID_USER,
                    EVENTS.SetAltitude,
                    (uint)newAltitude,
                    GROUPS.Default,
                    SIMCONNECT_EVENT_FLAG.DEFAULT
                );

                // Update our cached value
                _currentAltitude = newAltitude;
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[SimConnect] Failed to adjust altitude: {ex.Message}");
                return false;
            }
        }
    }

    /// <summary>
    /// Adjust vertical speed bug by delta (typically 100fpm increments)
    /// </summary>
    /// <param name="delta">FPM to add/subtract</param>
    /// <returns>True if command was sent successfully</returns>
    public bool AdjustVerticalSpeed(int delta)
    {
        lock (_lock)
        {
            if (!_isConnected)
            {
                Console.WriteLine("[SimConnect] Not connected - discarding VS command");
                return false;
            }

            if (!_verticalSpeedValid)
            {
                Console.WriteLine("[SimConnect] Current VS unknown - cannot apply delta");
                return false;
            }

            try
            {
                // Calculate new VS (clamp to reasonable values: -6000 to +6000 fpm)
                int newVS = _currentVerticalSpeed + delta * 100;
                if (newVS < -6000) newVS = -6000;
                if (newVS > 6000) newVS = 6000;

                Console.WriteLine($"[SimConnect] VS: {_currentVerticalSpeed}fpm + {delta * 100:+#;-#;0}fpm = {newVS}fpm");

                // Send VS bug set event (needs to be converted to signed value)
                // For negative values, use two's complement for 32-bit integer
                uint vsValue = (uint)newVS;

                _simConnect?.TransmitClientEvent(
                    SimConnect.SIMCONNECT_OBJECT_ID_USER,
                    EVENTS.SetVerticalSpeed,
                    vsValue,
                    GROUPS.Default,
                    SIMCONNECT_EVENT_FLAG.DEFAULT
                );

                // Update our cached value
                _currentVerticalSpeed = newVS;
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[SimConnect] Failed to adjust vertical speed: {ex.Message}");
                return false;
            }
        }
    }

    // Event handlers
    private void OnRecvOpen(SimConnect sender, SIMCONNECT_RECV_OPEN data)
    {
        lock (_lock)
        {
            _isConnected = true;
            Console.WriteLine($"[SimConnect] Connected to MSFS (v{data.dwApplicationVersionMajor}.{data.dwApplicationVersionMinor})");
        }
    }

    private void OnRecvQuit(SimConnect sender, SIMCONNECT_RECV data)
    {
        lock (_lock)
        {
            _isConnected = false;
            _headingValid = false;
            _altitudeValid = false;
            _verticalSpeedValid = false;
            _apStatusValid = false;
            Console.WriteLine("[SimConnect] MSFS closed - disconnected");
        }
    }

    private void OnRecvException(SimConnect sender, SIMCONNECT_RECV_EXCEPTION data)
    {
        SIMCONNECT_EXCEPTION eException = (SIMCONNECT_EXCEPTION)data.dwException;
        Console.WriteLine($"[SimConnect] Exception: {eException}");
    }

    private void OnRecvSimobjectData(SimConnect sender, SIMCONNECT_RECV_SIMOBJECT_DATA data)
    {
        if (data.dwRequestID == (uint)REQUESTS.AutopilotDataRequest)
        {
            var apData = (AutopilotData)data.dwData[0];
            lock (_lock)
            {
                _currentHeading = (int)Math.Round(apData.HeadingBug);
                _headingValid = true;

                _currentAltitude = (int)Math.Round(apData.AltitudeBug);
                _altitudeValid = true;

                _currentVerticalSpeed = (int)Math.Round(apData.VerticalSpeedBug);
                _verticalSpeedValid = true;

                // Update autopilot status flags
                _apMasterEngaged = apData.ApMaster != 0;
                _apHeadingActive = apData.ApHeadingLock != 0;
                _apAltitudeActive = apData.ApAltitudeLock != 0;
                _apVerticalSpeedActive = apData.ApVerticalHold != 0;
                _apStatusValid = true;
            }
        }
    }

    public void Dispose()
    {
        lock (_lock)
        {
            if (_simConnect != null)
            {
                _simConnect.Dispose();
                _simConnect = null;
                _isConnected = false;
                _headingValid = false;
                _altitudeValid = false;
                _verticalSpeedValid = false;
                _apStatusValid = false;
                Console.WriteLine("[SimConnect] Disconnected");
            }
        }
    }
}
