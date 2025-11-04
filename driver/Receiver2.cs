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
        private const byte PROTO_START = 0xAA;
        private const byte CMD_HDG_RESET = 0x10;
        private const byte CMD_HDG_SET = 0x11;
        private const int FRAME_SIZE = 4;

        private static SerialPort _serialPort;
        private static CancellationTokenSource _cts;

        // Statistics for debugging
        private static int _validFrames = 0;
        private static int _checksumErrors = 0;
        private static int _unknownCommands = 0;

        static async Task Main(string[] args)
        {
            Console.WriteLine("=== 4-Byte Protocol UART Receiver ===\n");

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
                Console.WriteLine("Listening for protocol frames... (Press Ctrl+C to exit)\n");

                // Setup cancellation for Ctrl+C
                _cts = new CancellationTokenSource();
                Console.CancelKeyPress += (sender, e) =>
                {
                    e.Cancel = true;
                    _cts.Cancel();
                };

                // Start async receiving with protocol parsing
                await ReceiveDataAsync(_cts.Token);
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
        /// Interprets the command and operand, displaying human-readable output
        /// </summary>
        private static void ParseAndDisplayCommand(byte command, byte operand)
        {
            // LEARNING POINT #4: Command Parsing
            // Each command has different operand interpretation

            string timestamp = DateTime.Now.ToString("HH:mm:ss.fff");

            switch (command)
            {
                case CMD_HDG_RESET:
                    // HDG:RESET always has operand = 0x00
                    Console.WriteLine($"[{timestamp}] HDG:RESET");
                    break;

                case CMD_HDG_SET:
                    // HDG:SET operand is a SIGNED 8-bit integer
                    // LEARNING POINT #5: Signed Integer Handling
                    // We receive unsigned byte, but interpret as signed (two's complement)
                    // Cast to sbyte to get the signed value (-128 to +127)
                    sbyte signedDelta = (sbyte)operand;
                    Console.WriteLine($"[{timestamp}] HDG:SET delta={signedDelta:+#;-#;0}");
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