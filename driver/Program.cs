using System;
using System.IO.Ports;

namespace SerialPortReader
{
    class Program
    {
        private static SerialPort serialPort = null;
        private const string DEFAULT_PORT = "COM4";
        private const int BAUD_RATE = 115200;

        static void Main(string[] args)
        {
            Console.WriteLine("================================");
            Console.WriteLine("Serial Port Data Reader");
            Console.WriteLine("================================\n");

            try
            {
                // Initialize Serial Port
                InitializeSerialPort(DEFAULT_PORT);

                Console.WriteLine("\nListening for data...");
                Console.WriteLine("Press ESC to exit.\n");

                // Main loop
                while (true)
                {
                    // Check for ESC key to exit
                    if (Console.KeyAvailable && Console.ReadKey(true).Key == ConsoleKey.Escape)
                    {
                        Console.WriteLine("\nExiting...");
                        break;
                    }

                    System.Threading.Thread.Sleep(10);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"\nError: {ex.Message}");
            }
            finally
            {
                Cleanup();
            }
        }

        static void InitializeSerialPort(string portName)
        {
            try
            {
                serialPort = new SerialPort(portName, BAUD_RATE, Parity.None, 8, StopBits.One);
                serialPort.DataReceived += SerialPort_DataReceived;
                serialPort.Open();
                Console.WriteLine($"Serial port {portName} opened ({BAUD_RATE} baud)");
            }
            catch (Exception ex)
            {
                throw new Exception($"Failed to open serial port {portName}: {ex.Message}");
            }
        }

        static void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string data = serialPort.ReadExisting();
                if (!string.IsNullOrEmpty(data))
                {
                    Console.Write(data);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"\nError reading serial data: {ex.Message}");
            }
        }

        static void Cleanup()
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
                serialPort.Dispose();
                Console.WriteLine("Serial port closed");
            }
        }
    }
}