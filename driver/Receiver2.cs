using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;

namespace SimpleUartReceiver
{
    class Program
    {
        // ===== PROTOCOL CONSTANTS (must match STM32 side) =====
        // RX Protocol: STM32 -> PC (4-byte frames, START=0xAA)
        private const byte PROTO_START = 0xAA;

        // TX Protocol: PC -> STM32 (3-byte frames, START=0x88)
        private const byte PROTO_START_TX = 0x88;
        private const byte CMD_LED_ON = 0x10;
        private const byte CMD_LED_OFF = 0x11;

        // Heading control (0x10-0x1F range)
        private const byte CMD_HDG_RESET = 0x10;
        private const byte CMD_HDG_SET = 0x11;

        // Altitude control (0x20-0x2F range)
        private const byte CMD_ALT_RESET = 0x20;
        private const byte CMD_ALT_SET = 0x21;

        // Vertical Speed control (0x30-0x3F range)
        private const byte CMD_VS_RESET = 0x30;
        private const byte CMD_VS_SET = 0x31;

        // Button control (0x50-0x5F range)
        private const byte CMD_BTN_AP_TOGGLE = 0x50;
        private const byte CMD_BTN_HDG_TOGGLE = 0x51;
        private const byte CMD_BTN_VS_TOGGLE = 0x52;
        private const byte CMD_BTN_ALT_TOGGLE = 0x53;

        // AP status control (0x60-0x6F range)
        private const byte CMD_SET_AP_STATUS = 0x60;

        private const int FRAME_SIZE = 4;

        private static SerialPort _serialPort;
        private static CancellationTokenSource _cts;

        // Statistics for debugging
        private static int _validFrames = 0;
        private static int _checksumErrors = 0;
        private static int _unknownCommands = 0;

        static async Task Main(string[] args)
        {
            Console.WriteLine("=== UART Bidirectional Protocol ===");
            Console.WriteLine("RX: 4-byte frames (STM32 -> PC)");
            Console.WriteLine("TX: 3-byte frames (PC -> STM32)\n");

            // Configure serial port (adjust COM port to match your system)
            _serialPort = new SerialPort
            {
                PortName = "COM4",        // Change to your port (COM3, COM4, etc.)
                BaudRate = 115200,
                DataBits = 8,
                Parity = Parity.None,
                StopBits = StopBits.One,
                Handshake = Handshake.None
            };

            try
            {
                // Open the port
                _serialPort.Open();
                Console.WriteLine($"Connected to {_serialPort.PortName} at {_serialPort.BaudRate} baud");
                Console.WriteLine("Bidirectional communication active!\n");
                Console.WriteLine("=== Commands ===");
                Console.WriteLine("Press '1' - Turn LED ON");
                Console.WriteLine("Press '0' - Turn LED OFF");
                Console.WriteLine("Press 'A' - Send AP Status (engaged, HDG+ALT active)");
                Console.WriteLine("Press 'B' - Send AP Status (disengaged)");
                Console.WriteLine("Press 'H' - Show help");
                Console.WriteLine("Press Ctrl+C - Exit\n");

                // Setup cancellation for Ctrl+C
                _cts = new CancellationTokenSource();
                Console.CancelKeyPress += (sender, e) =>
                {
                    e.Cancel = true;
                    _cts.Cancel();
                };

                // Run both receive and console input tasks concurrently
                Task receiveTask = ReceiveDataAsync(_cts.Token);
                Task consoleTask = HandleConsoleInputAsync(_cts.Token);

                // Wait for either task to complete (or cancellation)
                await Task.WhenAny(receiveTask, consoleTask);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: {ex.Message}");
            }
            finally
            {
                // Cleanup
                if (_serialPort?.IsOpen == true)
                {
                    _serialPort.Close();
                }

                // Print statistics
                Console.WriteLine("\n=== Statistics ===");
                Console.WriteLine($"Valid frames:     {_validFrames}");
                Console.WriteLine($"Checksum errors:  {_checksumErrors}");
                Console.WriteLine($"Unknown commands: {_unknownCommands}");
                Console.WriteLine("\nDisconnected.");
            }
        }

        /// <summary>
        /// Main receive loop with protocol parsing
        /// </summary>
        private static async Task ReceiveDataAsync(CancellationToken cancellationToken)
        {
            // LEARNING POINT #1: Circular Buffer Approach
            // We use a List as a dynamic buffer that accumulates incoming bytes.
            // Serial data is a STREAM, not discrete packets, so we need to:
            // 1. Collect bytes as they arrive
            // 2. Search for START byte (0xAA) - this is "frame synchronization"
            // 3. Extract and validate complete frames
            // 4. Remove processed bytes from buffer

            List<byte> receiveBuffer = new List<byte>();
            byte[] readBuffer = new byte[32];  // Read up to 32 bytes at a time

            while (!cancellationToken.IsCancellationRequested)
            {
                try
                {
                    // STEP 1: Read available bytes asynchronously
                    int bytesRead = await _serialPort.BaseStream.ReadAsync(
                        readBuffer, 0, readBuffer.Length, cancellationToken);

                    if (bytesRead > 0)
                    {
                        // STEP 2: Add new bytes to our circular buffer
                        for (int i = 0; i < bytesRead; i++)
                        {
                            receiveBuffer.Add(readBuffer[i]);
                        }

                        // STEP 3: Try to extract and process frames from buffer
                        // Keep processing until no more complete frames are available
                        while (TryProcessFrame(receiveBuffer))
                        {
                            // TryProcessFrame returns true if a frame was processed
                            // Loop continues to process multiple frames if available
                        }
                    }
                }
                catch (OperationCanceledException)
                {
                    // Expected when Ctrl+C is pressed
                    break;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"\nError reading data: {ex.Message}");
                    break;
                }
            }
        }

        /// <summary>
        /// Attempts to find and process a complete frame from the buffer
        /// </summary>
        /// <returns>True if a frame was processed, false if no complete frame found</returns>
        private static bool TryProcessFrame(List<byte> buffer)
        {
            // LEARNING POINT #2: Frame Synchronization
            // We need at least 4 bytes to have a complete frame
            if (buffer.Count < FRAME_SIZE)
                return false;

            // STEP 1: Find the START byte (0xAA)
            // This is called "frame synchronization" - finding where a packet begins
            int startIndex = buffer.IndexOf(PROTO_START);

            if (startIndex == -1)
            {
                // No START byte found - discard all bytes (they're garbage)
                buffer.Clear();
                return false;
            }

            // STEP 2: Remove any garbage bytes before START
            if (startIndex > 0)
            {
                buffer.RemoveRange(0, startIndex);
            }

            // STEP 3: Check if we have a complete frame (4 bytes starting with START)
            if (buffer.Count < FRAME_SIZE)
            {
                // Not enough bytes yet, wait for more data
                return false;
            }

            // STEP 4: Extract the frame
            byte start = buffer[0];      // 0xAA
            byte command = buffer[1];    // Command ID
            byte operand = buffer[2];    // Command parameter
            byte checksum = buffer[3];   // XOR checksum

            // STEP 5: Validate checksum
            // LEARNING POINT #3: Checksum Validation
            // The checksum is calculated as: START ^ COMMAND ^ OPERAND
            // This catches most transmission errors
            byte calculatedChecksum = (byte)(start ^ command ^ operand);

            if (checksum != calculatedChecksum)
            {
                // Checksum mismatch - frame is corrupted
                _checksumErrors++;
                Console.WriteLine($"[ERROR] Checksum mismatch: expected {calculatedChecksum:X2}, got {checksum:X2}");

                // Remove the bad START byte and try to resync on next START
                buffer.RemoveAt(0);
                return true;  // Return true to keep searching for next frame
            }

            // STEP 6: Frame is valid - remove it from buffer
            buffer.RemoveRange(0, FRAME_SIZE);
            _validFrames++;

            // STEP 7: Parse and display the command
            ParseAndDisplayCommand(command, operand);

            return true;  // Frame processed, check for more frames
        }

        /// <summary>
        /// Sends a 3-byte command to STM32 (PC -> STM32 protocol)
        /// Frame format: [START=0x88, COMMAND, CHECKSUM]
        /// </summary>
        private static void SendCommand(byte command)
        {
            SendCommandWithOperand(command, 0x00);
        }

        /// <summary>
        /// Sends a 3-byte command with operand to STM32 (PC -> STM32 protocol)
        /// Frame format: [START=0x88, COMMAND, CHECKSUM]
        /// Note: Protocol uses 3-byte frames, so operand is sent as command parameter
        /// </summary>
        private static void SendCommandWithOperand(byte command, byte operand)
        {
            byte[] frame = new byte[3];
            frame[0] = PROTO_START_TX;  // START = 0x88
            frame[1] = command;          // COMMAND (operand encoded in command byte for 3-byte protocol)
            frame[2] = (byte)(PROTO_START_TX ^ command);  // CHECKSUM = START ^ COMMAND

            try
            {
                _serialPort.Write(frame, 0, frame.Length);
                string cmdName = command switch
                {
                    CMD_LED_ON => "LED_ON",
                    CMD_LED_OFF => "LED_OFF",
                    CMD_SET_AP_STATUS => $"SET_AP_STATUS(0x{operand:X2})",
                    _ => $"0x{command:X2}"
                };
                Console.WriteLine($"[SENT] {cmdName} -> [0x{frame[0]:X2}, 0x{frame[1]:X2}, 0x{frame[2]:X2}]");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[ERROR] Failed to send command: {ex.Message}");
            }
        }

        /// <summary>
        /// Handles console input for sending LED commands
        /// </summary>
        private static async Task HandleConsoleInputAsync(CancellationToken cancellationToken)
        {
            await Task.Run(() =>
            {
                while (!cancellationToken.IsCancellationRequested)
                {
                    try
                    {
                        if (Console.KeyAvailable)
                        {
                            var key = Console.ReadKey(intercept: true);

                            switch (key.Key)
                            {
                                case ConsoleKey.D1:
                                case ConsoleKey.NumPad1:
                                    SendCommand(CMD_LED_ON);
                                    break;

                                case ConsoleKey.D0:
                                case ConsoleKey.NumPad0:
                                    SendCommand(CMD_LED_OFF);
                                    break;

                                case ConsoleKey.A:
                                    // Send AP engaged with HDG and ALT active
                                    // Bitfield: 0x01 (engaged) | 0x02 (HDG) | 0x04 (ALT) = 0x07
                                    SendCommandWithOperand(CMD_SET_AP_STATUS, 0x07);
                                    break;

                                case ConsoleKey.B:
                                    // Send AP disengaged (all modes off)
                                    SendCommandWithOperand(CMD_SET_AP_STATUS, 0x00);
                                    break;

                                case ConsoleKey.H:
                                    Console.WriteLine("\n=== Commands ===");
                                    Console.WriteLine("Press '1' - Turn LED ON");
                                    Console.WriteLine("Press '0' - Turn LED OFF");
                                    Console.WriteLine("Press 'A' - Send AP Status (engaged, HDG+ALT active)");
                                    Console.WriteLine("Press 'B' - Send AP Status (disengaged)");
                                    Console.WriteLine("Press 'H' - Show this help");
                                    Console.WriteLine("Press Ctrl+C - Exit\n");
                                    break;
                            }
                        }
                        Thread.Sleep(50);  // Small delay to prevent CPU spinning
                    }
                    catch (OperationCanceledException)
                    {
                        break;
                    }
                }
            }, cancellationToken);
        }

        /// <summary>
        /// Interprets the command and operand, displaying human-readable output
        /// </summary>
        private static void ParseAndDisplayCommand(byte command, byte operand)
        {
            // LEARNING POINT #4: Command Parsing
            // Each command has different operand interpretation

            string timestamp = DateTime.Now.ToString("HH:mm:ss.fff");

            switch (command)
            {
                // ===== HEADING CONTROL =====
                case CMD_HDG_RESET:
                    // HDG:RESET always has operand = 0x00
                    Console.WriteLine($"[{timestamp}] HDG:RESET");
                    break;

                case CMD_HDG_SET:
                    // HDG:SET operand is a SIGNED 8-bit integer
                    // LEARNING POINT #5: Signed Integer Handling
                    // We receive unsigned byte, but interpret as signed (two's complement)
                    // Cast to sbyte to get the signed value (-128 to +127)
                    sbyte hdgDelta = (sbyte)operand;
                    Console.WriteLine($"[{timestamp}] HDG:SET delta={hdgDelta:+#;-#;0}");
                    break;

                // ===== ALTITUDE CONTROL =====
                case CMD_ALT_RESET:
                    // ALT:RESET always has operand = 0x00
                    Console.WriteLine($"[{timestamp}] ALT:RESET");
                    break;

                case CMD_ALT_SET:
                    // ALT:SET operand is a SIGNED 8-bit integer
                    sbyte altDelta = (sbyte)operand;
                    Console.WriteLine($"[{timestamp}] ALT:SET delta={altDelta:+#;-#;0}");
                    break;

                // ===== VERTICAL SPEED CONTROL =====
                case CMD_VS_RESET:
                    // VS:RESET always has operand = 0x00
                    Console.WriteLine($"[{timestamp}] VS:RESET");
                    break;

                case CMD_VS_SET:
                    // VS:SET operand is a SIGNED 8-bit integer
                    sbyte vsDelta = (sbyte)operand;
                    Console.WriteLine($"[{timestamp}] VS:SET delta={vsDelta:+#;-#;0}");
                    break;

                // ===== BUTTON CONTROL =====
                case CMD_BTN_AP_TOGGLE:
                    // AP button toggle (operand always 0x00)
                    Console.WriteLine($"[{timestamp}] BTN:AP_TOGGLE");
                    break;

                case CMD_BTN_HDG_TOGGLE:
                    // HDG button toggle (operand always 0x00)
                    Console.WriteLine($"[{timestamp}] BTN:HDG_TOGGLE");
                    break;

                case CMD_BTN_VS_TOGGLE:
                    // VS button toggle (operand always 0x00)
                    Console.WriteLine($"[{timestamp}] BTN:VS_TOGGLE");
                    break;

                case CMD_BTN_ALT_TOGGLE:
                    // ALT button toggle (operand always 0x00)
                    Console.WriteLine($"[{timestamp}] BTN:ALT_TOGGLE");
                    break;

                default:
                    // Unknown command - log warning but continue
                    _unknownCommands++;
                    Console.WriteLine($"[{timestamp}] [WARN] Unknown command: 0x{command:X2}, operand: 0x{operand:X2}");
                    break;
            }
        }
    }
}