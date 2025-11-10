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
        SetHeading
    }

    private enum GROUPS
    {
        Default
    }

    // Data structure for autopilot heading
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct AutopilotData
    {
        public double HeadingBug;  // AUTOPILOT HEADING LOCK DIR
    }

    public bool IsConnected
    {
        get { lock (_lock) return _isConnected; }
    }

    public int CurrentHeading
    {
        get { lock (_lock) return _currentHeading; }
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

                // Define data structure for autopilot heading
                _simConnect.AddToDataDefinition(
                    DEFINITIONS.AutopilotData,
                    "AUTOPILOT HEADING LOCK DIR",
                    "degrees",
                    SIMCONNECT_DATATYPE.FLOAT64,
                    0.0f,
                    SimConnect.SIMCONNECT_UNUSED
                );

                _simConnect.RegisterDataDefineStruct<AutopilotData>(DEFINITIONS.AutopilotData);

                // Map event for setting heading
                _simConnect.MapClientEventToSimEvent(EVENTS.SetHeading, "HEADING_BUG_SET");

                // Add event to notification group
                _simConnect.AddClientEventToNotificationGroup(GROUPS.Default, EVENTS.SetHeading, false);
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
                Console.WriteLine("[SimConnect] Disconnected");
            }
        }
    }
}
