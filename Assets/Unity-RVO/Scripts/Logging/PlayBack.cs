using System.IO;
using UnityEngine;
using Unity.Collections;
using System.Collections.Generic;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace RVO {
    public class PlayBack : MonoBehaviour
    {
        [System.Serializable]
        public struct PlaybackFrame {
            public int frame;
            public float simulation_time;
            public bool[] active;
            public bool[] is_agent;
            public float3[] positions;
            public quaternion[] rotations;
            public float3[] velocities;
            public float3[] destinations;
            public float[] radii;
            public int[] num_neighbors;
        }

        public TextAsset logFile = null;
        private TextAsset _prevLogFile = null;
        [SerializeField] private Camera scene_cam;

        public int simulation_start_frame = 0;
        public float simulation_start_time = 0f;
        public Vector2 bounds = Vector2.zero;
        public int num_total_agents = 0;
        public int num_agents = 0;
        public int num_non_agents = 0;
        public int num_frames = 0;
        public float duration = 0f;
        private List<PlaybackFrame> frames = new();

        private bool _playing = false;
        private float _playbackTime = 0f;
        private float _playbackSpeed = 1f;
        [SerializeField] private GameObject _robotPrefab;
        private List<GameObject> _spawnedRobots = new();
        [SerializeField] private GameObject _nonAgentPrefab;
        private List<GameObject> _spawnedNonAgents = new();

        public void LoadLogData() {
            // Cancel out if we don't even have a file
            if (logFile == null) {
                Debug.LogError("Log file not set!");
                return;
            }

            // Initialize reader
            using MemoryStream stream = new MemoryStream(logFile.bytes);
            using BinaryReader reader = new BinaryReader(stream);

            // Get from the initial first lines: 
            simulation_start_frame = reader.ReadInt32();    // The simulation start frame since the start of the game/program
            simulation_start_time = reader.ReadSingle();    // The simulation start time since the start of the game/program
            bounds = new Vector2(                           // The size bounds along the XZ axes
                reader.ReadSingle(),
                reader.ReadSingle()
            );
            num_total_agents = reader.ReadInt32();          // the total number of agents
            num_agents = reader.ReadInt32();                // the number of active VO/RVO/HRVO agents
            num_non_agents = reader.ReadInt32();            // the number of NonAgents

            // Iterate through each frame until the end
            frames = new();
            duration = 0f;
            while (reader.BaseStream.Position < reader.BaseStream.Length) {
                frames.Add(ReadFrame(reader));
            }
            num_frames = frames.Count;
            Debug.Log($"{num_frames} frames processed; Total duration = {duration} seconds");

            // Set some playbacks
            _playing = false;
            _playbackTime = 0f;
            _playbackSpeed = 1f;

            // After reading the data, and if we have a camera, let's re-mount it
            if (scene_cam != null) {
                Vector3 cam_pos = new Vector3(bounds.x / 2f, 100f, bounds.y / 2f);
                float screen_ratio = (float)Screen.width / (float)Screen.height;
                float target_ratio = bounds.x / bounds.y;
                float ortho_size = (screen_ratio >= target_ratio)
                    ? bounds.y / 2
                    : bounds.y / 2 * (target_ratio / screen_ratio);

                scene_cam.transform.position = cam_pos;
                scene_cam.transform.rotation = Quaternion.LookRotation(Vector3.down, Vector3.up);
                scene_cam.orthographic = true;
                scene_cam.orthographicSize = ortho_size;
            }

            // If we're playing, we need to spawn our agents
            if (Application.isPlaying) {
                GenerateAgents();
                GenerateNonAgents();
            }
        }

        private void GenerateAgents() {
            // Generate `num_total_agents` amount of robots.

            // if we spawned any agents, might as well keep them
            if (_spawnedRobots.Count > 0) {
                // Destroy and cull out extra agents
                while(_spawnedRobots.Count > num_agents) {
                    int endIndex = _spawnedRobots.Count-1;
                    Destroy(_spawnedRobots[endIndex]);
                    _spawnedRobots.RemoveAt(endIndex);
                }
            }

            // Now attempt to add new number of agents
            while(_spawnedRobots.Count < num_agents) {
                GameObject newRobot = Instantiate(_robotPrefab);
                _spawnedRobots.Add(newRobot);
            }
        }

        private void GenerateNonAgents() {
            // Generate `num_total_agents` amount of robots.

            // if we spawned any agents, might as well keep them
            if (_spawnedNonAgents.Count > 0) {
                // Destroy and cull out extra agents
                while(_spawnedNonAgents.Count > num_non_agents) {
                    int endIndex = _spawnedNonAgents.Count-1;
                    Destroy(_spawnedNonAgents[endIndex]);
                    _spawnedNonAgents.RemoveAt(endIndex);
                }
            }

            // Now attempt to add new number of agents
            while(_spawnedNonAgents.Count < num_non_agents) {
                GameObject newNonAgent = Instantiate(_nonAgentPrefab);
                _spawnedNonAgents.Add(newNonAgent);
            }
        }

        public PlaybackFrame ReadFrame(BinaryReader reader) {
            
            PlaybackFrame frame = new PlaybackFrame {
                frame = reader.ReadInt32(),
                simulation_time = reader.ReadSingle(),
                active = new bool[num_total_agents],
                is_agent = new bool[num_total_agents],
                positions = new float3[num_total_agents],
                rotations = new quaternion[num_total_agents],
                velocities = new float3[num_total_agents],
                destinations = new float3[num_total_agents],
                radii = new float[num_total_agents],
                num_neighbors = new int[num_total_agents]
            };

            ReadNativeArray<bool>(reader, ref frame.active);
            ReadNativeArray<bool>(reader, ref frame.is_agent);
            ReadNativeArray<float3>(reader, ref frame.positions);
            ReadNativeArray<quaternion>(reader, ref frame.rotations);
            ReadNativeArray<float3>(reader, ref frame.velocities);
            ReadNativeArray<float3>(reader, ref frame.destinations);
            ReadNativeArray<float>(reader, ref frame.radii);
            ReadNativeArray<int>(reader, ref frame.num_neighbors);

            duration = Mathf.Max(duration, frame.simulation_time);

            return frame;
        }

        unsafe static void ReadNativeArray<T>( BinaryReader reader, ref T[] array) where T : unmanaged {
            int byteCount = array.Length * UnsafeUtility.SizeOf<T>();
            byte[] bytes = reader.ReadBytes(byteCount);
            NativeArray<T> native_array = new NativeArray<T>(array.Length, Allocator.Temp);

            fixed (byte* src = bytes) {
                UnsafeUtility.MemCpy(
                    native_array.GetUnsafePtr(),
                    src,
                    byteCount);
            }
            native_array.CopyTo(array);
        }

        public void ResetLogData() {
            frames = new();
            num_total_agents = 0;
            num_agents = 0;
            num_non_agents = 0;
            num_frames = 0;
            duration = 0f;
        }

        private void OnValidate() {
            if (_prevLogFile != logFile) {
                _prevLogFile = logFile;
                if (logFile != null) LoadLogData();
                else ResetLogData();
            }
        }

        private void Awake() {
            if (frames.Count > 0) GenerateAgents();
        }

        private void Update() {
            // If we're not playing or if our instance is not set, then end early
            if (frames.Count == 0 || !_playing) return;
            // Adjust the current timestamp based on delta time
            _playbackTime += Time.deltaTime * _playbackSpeed;
            // Loop back to 0 if we reach the end
            if (_playbackTime > duration) _playbackTime = 0f;

            // Get the frame corresponding to the playback time
            if (TryGetTimestampIndex(_playbackTime, out int frame_index)) {
                PlaybackFrame frame = frames[frame_index];
                // Update each agent
                for(int i = 0; i < num_agents; i++) {
                    GameObject robot = _spawnedRobots[i];
                    robot.SetActive(frame.active[i]);
                    robot.transform.localPosition = frame.positions[i];
                    robot.transform.localRotation = frame.rotations[i];
                }
                // Update each non-agent
                for(int j = 0; j < num_non_agents; j++) {
                    GameObject nonAgent = _spawnedNonAgents[j];
                    int j2 = num_agents + j;
                    nonAgent.SetActive(frame.active[j2]);
                    nonAgent.transform.localPosition = frame.positions[j2];
                    nonAgent.transform.localRotation = frame.rotations[j2];
                    nonAgent.transform.localScale = Vector3.one * frame.radii[j2];
                }
            }
        }


        public virtual bool TryGetTimestampIndex(float t, out int index) {
            // Default: set index = 0
            index = 0;
            
            //  terminate early if we don't have frames
            if (frames == null || frames.Count == 0) return false;

            // Handle before first sample
            if (t <= frames[0].simulation_time) return false;
            
            // Handle after last sample
            if (t >= frames[^1].simulation_time) {
                index = frames.Count - 1;
                return false;   
            }

            // We're now somewhere between this trajectory's first and last timestamp
            // So now we must interpolate and find the index where the timestamp fits between
            for (int i = 0; i < frames.Count - 1; i++) {
                // Grab the current and the next trajectory point
                var a = frames[i];
                var b = frames[i + 1];
                // Check
                if (t >= a.simulation_time && t <= b.simulation_time) {
                    index = i;
                    return true;
                }
            }
            // In the worst case, just return false
            return false;
        }


        void OnGUI() {
            // Define width and height + begin the overall GUI window in the inspector
            const int panelWidth = 240;
            const int panelHeight = 150;
            GUILayout.BeginArea(new Rect(10f, 10f, panelWidth, panelHeight), "Playback", GUI.skin.box);

            // Play/Pause Controls            
            GUILayout.BeginHorizontal();

            GUILayout.BeginVertical();
            GUI.enabled = !_playing;
            if (GUILayout.Button("Play")) _playing = true;
            GUI.enabled = true;
            GUILayout.EndVertical();

            GUILayout.BeginVertical();
            GUI.enabled = _playing;
            if (GUILayout.Button("Pause")) _playing = false;
            GUI.enabled = true;
            GUILayout.EndVertical();

            GUILayout.EndHorizontal();

            // Scrollbar to control the player
            GUILayout.Label("Time");
            _playbackTime = GUILayout.HorizontalSlider(_playbackTime, 0f, duration);

            GUILayout.Label($"Speed: {_playbackSpeed:0.1f}");
            _playbackSpeed = GUILayout.HorizontalSlider(_playbackSpeed, 0.1f, 5f);

            GUILayout.EndArea();
        }
        
    }
}
