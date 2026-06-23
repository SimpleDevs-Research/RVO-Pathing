using System.IO;
using UnityEngine;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace RVO {
    [System.Serializable]
    public class VOLogger
    {

        [Tooltip("Which directory, in persistent memory, do we save files into?")]     
        public string dirName = "";
        [Tooltip("What should we name the file?")]
        public string fileName = "";
        [Tooltip("Do we append '0' to the filename?")]
        public bool append_zero_to_filename = true;

        private BinaryWriter _writer;
        private Generator _generator;
        private VO_OP _vo_op;
        private string filePath;
        private bool _is_writing = false;
        public bool is_writing => _is_writing;

        public void StartRecording(Generator generator) {
            // End early if already writing
            if (_is_writing) {
                Debug.LogError("Cannot start recording; already writing!");
                return;
            }

            // Set References
            _generator = generator;
            _vo_op = _generator.vo_op;

            // Create output directory
            string dname = $"{Application.persistentDataPath}/{GetCurrentDateTime()}";
            if (dirName != null && dirName.Length > 0) {
                dname = $"{Application.persistentDataPath}/{dirName}";
            }
            CheckOrCreateDirectory(dname);

            // Determine the filename and ressulting filepath
            string fname = (fileName != null && fileName.Length > 0) ? fileName : GetCurrentDateTime();
            filePath = (append_zero_to_filename) ? Path.Combine(dname, fname+"_0.bytes") : Path.Combine(dname, fname+".bytes");

            // Add numbers to end of filepath if duplicates are found.
            int counter = 1;
            while(File.Exists(filePath)) {
                filePath = Path.Combine(dname, fname+$"_{counter}.bytes");
                counter++;
            }

            // Initialize writer
            _writer = new BinaryWriter(File.Open(filePath, FileMode.Create));
            _is_writing = true;

            // We write the first few lines:
            _writer.Write(_generator.simulation_start_frame);   // When the scene was started, since the start of the game.
            _writer.Write(_generator.simulation_start_time);    // When the scene was started, since when the game started.
            _writer.Write(_generator.bounds.x);                 // 2 floats representing the X and Z size of the simulation space
            _writer.Write(_generator.bounds.y);
            _writer.Write(_generator.num_total_agents);         // Total number of agents
            _writer.Write(_generator.num_agents);               // # of VO/RVO/HRVO agents
            _writer.Write(_generator.num_non_agents);           // # of NonAgents
        }

        public void StopRecording() {
            // End early if not writing
            if (!_is_writing) {
                Debug.Log("No need to stop recording; the writer is already unset");
                return;
            }
            // CLose writer
            _writer.Close();
            _is_writing = false;
        }

        public void RecordFrame() {
            // Write the header
            _writer.Write(Time.frameCount);
            _writer.Write(_generator.simulation_time);

            // Write the necessary native arrays
            WriteNativeArray<bool>(_writer, _vo_op.active);
            WriteNativeArray<bool>(_writer,_vo_op.is_agent);
            WriteNativeArray<float3>(_writer,_vo_op.positions);
            WriteNativeArray<quaternion>(_writer,_vo_op.rotations);
            WriteNativeArray<float3>(_writer,_vo_op.velocities);
            WriteNativeArray<float3>(_writer,_vo_op.destinations);
            WriteNativeArray<int>(_writer,_vo_op.num_neighbors);
        }

        unsafe static void WriteNativeArray<T>( BinaryWriter writer, NativeArray<T> array ) where T : unmanaged {
            int bytes = array.Length * UnsafeUtility.SizeOf<T>();
            byte[] buffer = new byte[bytes];

            fixed (byte* dst = buffer) {
                UnsafeUtility.MemCpy(
                    dst,
                    array.GetUnsafeReadOnlyPtr(),
                    bytes
                );
            }

            writer.Write(buffer);
        }

        private static string GetCurrentDateTime(string format = "HH-mm-ss") {
            return System.DateTime.Now.ToString(format);
        }

        private static bool CheckDirectoryExists(string dirPath) {
            return Directory.Exists(dirPath);
        }

        private static bool CheckOrCreateDirectory(string dirPath) {
            if (!CheckDirectoryExists(dirPath)) Directory.CreateDirectory(dirPath);
            return true;
        }
    }
}
