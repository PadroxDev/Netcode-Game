using Cinemachine;
using System.Collections.Generic;
using Unity.Netcode;
using UnityEngine;

namespace Padrox
{
    public struct InputPayload : INetworkSerializable {
        public int Tick;
        public Vector3 InputVector;

        public void NetworkSerialize<T>(BufferSerializer<T> serializer) where T : IReaderWriter {
            serializer.SerializeValue(ref Tick);
            serializer.SerializeValue(ref InputVector);
        }
    }

    public struct StatePayload : INetworkSerializable {
        public int Tick;
        public Vector3 Position;
        public Quaternion Rotation;
        public Vector3 Velocity;
        public Vector3 AngularVelocity;

        public void NetworkSerialize<T>(BufferSerializer<T> serializer) where T : IReaderWriter {
            serializer.SerializeValue(ref Tick);
            serializer.SerializeValue(ref Position);
            serializer.SerializeValue(ref Rotation);
            serializer.SerializeValue(ref Velocity);
            serializer.SerializeValue(ref AngularVelocity);
        }
    }

    public class PlayerController : NetworkBehaviour
    {
        const float SERVER_TICK_RATE = 60f;
        const int BUFFER_SIZE = 1024;

        [ExposedScriptableObject]
        [SerializeField] private PlayerData _data;
        [SerializeField] private float _reconciliationThreshold = 10f;

        [SerializeField] private Transform _serverCube;
        [SerializeField] private Transform _clientCube;

        [Space, Header("References")]
        [SerializeField] private Transform _orientation;
        [SerializeField] private Transform _groundCheck;
        [SerializeField] private CinemachineFreeLook _playerCam;
        [SerializeField] private AudioListener _playerAudioListener;

        private Rigidbody _rb;
        private Vector2 _inputDir;
        private Vector3 _moveDir;
        private bool _grounded;
        private bool _isOnSlope;
        private RaycastHit _slopeHit;

        // Netcode client specific
        NetworkTimer _timer;
        CircularBuffer<StatePayload> _clientStateBuffer;
        CircularBuffer<InputPayload> _clientInputBuffer;
        StatePayload _lastServerState;
        StatePayload _lastProcessedState;

        // Netcode server specific
        CircularBuffer<StatePayload> _serverStateBuffer;
        Queue<InputPayload> _serverInputQueue;

        private void Awake() {
            _rb = GetComponent<Rigidbody>();

            _inputDir = Vector2.zero;
            _moveDir = Vector3.zero;

            _timer = new NetworkTimer(SERVER_TICK_RATE);
            _clientStateBuffer = new CircularBuffer<StatePayload>(BUFFER_SIZE);
            _clientInputBuffer = new CircularBuffer<InputPayload>(BUFFER_SIZE);

            _serverStateBuffer = new CircularBuffer<StatePayload>(BUFFER_SIZE);
            _serverInputQueue = new Queue<InputPayload>();
        }

        public override void OnNetworkSpawn() {
            if (!IsOwner) {
                _playerAudioListener.enabled = false;
                _playerCam.Priority = 0;
                return;
            }

            _playerAudioListener.enabled = true;
            _playerCam.Priority = 100;
        }

        private void Start() {
            if (!IsOwner) return;
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }

        private void Update() {
            _timer.Update(Time.deltaTime);

            if(IsOwner && Input.GetKeyDown(KeyCode.E)) {
                _rb.AddForce(transform.forward * 200f, ForceMode.Impulse);
                Debug.Log("Fast !");
            }
        }

        private void FixedUpdate() {
            if (!IsOwner) return;

            while(_timer.ShouldTick()) {
                HandleClientTick();
                HandleServerTick();
            }
        }

        private void HandleServerTick() {
            int bufferIndex = -1;
            while(_serverInputQueue.Count > 0) {
                InputPayload inputPayload = _serverInputQueue.Dequeue();
                bufferIndex = inputPayload.Tick % BUFFER_SIZE;

                StatePayload statePayload = SimulateMovement(inputPayload);
                _serverCube.position = statePayload.Position + Vector3.up * 2.5f;
                _serverStateBuffer.Add(statePayload, bufferIndex);
            }

            if (bufferIndex == -1) return;
            SendToClientRpc(_serverStateBuffer.Get(bufferIndex));
        }

        StatePayload SimulateMovement(InputPayload inputPayload) {
            Physics.simulationMode = SimulationMode.Script;

            Move(inputPayload.InputVector);
            Physics.Simulate(Time.fixedDeltaTime);
            Physics.simulationMode = SimulationMode.FixedUpdate;

            return new StatePayload() {
                Tick = inputPayload.Tick,
                Position = transform.position,
                Rotation = transform.rotation,
                Velocity = _rb.velocity,
                AngularVelocity = _rb.angularVelocity
            };
        }

        [ClientRpc]
        private void SendToClientRpc(StatePayload statePayload) {
            if (!IsOwner) return;
            _lastProcessedState = statePayload;
        }

        private void HandleClientTick() {
            if (!IsClient) return;

            int currentTick = _timer.CurrentTick;
            int bufferIndex = currentTick % BUFFER_SIZE;
            Vector2 inputVector = InputManager.GetMoveDirection();

            InputPayload inputPayload = new InputPayload() {
                Tick = currentTick,
                InputVector = inputVector
            };

            _clientInputBuffer.Add(inputPayload, bufferIndex);
            SendToServerRpc(inputPayload);

            StatePayload statePayload = ProcessMovement(inputPayload);
            _clientCube.position = statePayload.Position + Vector3.up * 2.5f;
            _clientStateBuffer.Add(statePayload, bufferIndex);

            HandleServerReconciliation();
        }

        private bool ShouldReconcile() {
            bool isNewServerState = !_lastServerState.Equals(default);
            bool isLastStateUndefinedOrDifferent = _lastProcessedState.Equals(default)
                                                   || !_lastProcessedState.Equals(_lastServerState);

            return isNewServerState && isLastStateUndefinedOrDifferent;
        }

        private void HandleServerReconciliation() {
            if (!ShouldReconcile()) return;
            {
                float positionError;
                int bufferIndex;
                StatePayload rewindState = default;

                bufferIndex = _lastProcessedState.Tick % BUFFER_SIZE;
                if (bufferIndex - 1 < 0) return; // Not enough information to reconcile

                // Host RPC execute immediately, so we can use the last server state
                rewindState = IsHost ? _serverStateBuffer.Get(bufferIndex - 1) : _lastServerState;
                positionError = Vector3.Distance(rewindState.Position, _clientStateBuffer.Get(bufferIndex).Position);

                if(positionError > _reconciliationThreshold) {
                    Debug.Break();
                    ReconcileState(rewindState);
                }

                _lastProcessedState = _lastServerState;
            }
        }

        private void ReconcileState(StatePayload rewindState) {
            transform.position = rewindState.Position;
            transform.rotation = rewindState.Rotation;
            _rb.velocity = rewindState.Velocity;
            _rb.angularVelocity = rewindState.AngularVelocity;

            if (!rewindState.Equals(_lastServerState)) return;

            _clientStateBuffer.Add(rewindState, rewindState.Tick);

            // Replay all inputs front the rewind state to the current state
            int tickToReplay = _lastServerState.Tick;
            while(tickToReplay < _timer.CurrentTick) {
                int bufferIndex = tickToReplay % BUFFER_SIZE;
                StatePayload statePayload = ProcessMovement(_clientInputBuffer.Get(bufferIndex));
                _clientStateBuffer.Add(statePayload, bufferIndex);
                tickToReplay++;
            }
        }

        [ServerRpc]
        private void SendToServerRpc(InputPayload input) {
            _serverInputQueue.Enqueue(input);
        }

        StatePayload ProcessMovement(InputPayload input) {
            Move(input.InputVector);

            return new StatePayload() {
                Tick = input.Tick,
                Position = transform.position,
                Rotation = transform.rotation,
                Velocity = _rb.velocity,
                AngularVelocity = _rb.angularVelocity
            };
        }

        private void Move(Vector2 inputDir) {
            GroundCheck();
            EvaluateMoveDir(inputDir);
            HandleMovement();
            HandleRotation();
            HandleDrag();
            SpeedRestraint();
        }

        private void GroundCheck() {
            _grounded = Physics.Raycast(_groundCheck.position, Vector3.down, _data.GroundCheckDistance, _data.WhatIsGround);
        }

        private void EvaluateMoveDir(Vector2 inputDir) {
            Vector3 camPos = new Vector3(_playerCam.transform.position.x, transform.position.y, _playerCam.transform.position.z);
            Vector3 viewDir = (transform.position - camPos).normalized;
            _orientation.forward = viewDir;

            _moveDir = _orientation.forward * inputDir.y + _orientation.right * inputDir.x;
        }

        private void HandleMovement() {
            _isOnSlope = OnSlope();
            _rb.useGravity = !_isOnSlope;

            if (_moveDir == Vector3.zero) return;

            Vector3 dir = _isOnSlope ? GetSlopeMoveDirection() : _moveDir;
            _rb.AddForce(dir * _data.MoveSpeed * 10f, ForceMode.Force);
        }

        private void HandleRotation() {
            if (_moveDir == Vector3.zero) return;
            transform.forward = Vector3.Slerp(transform.forward, _moveDir.normalized, Time.fixedDeltaTime * _data.RotationSpeed);
        }

        private void HandleDrag() {
            _rb.drag = _grounded ? _data.GroundDrag : _data.AirDrag;
        }

        private void SpeedRestraint() {
            Vector3 vel;
            if (_isOnSlope) {
                vel = _rb.velocity;
            } else { // Flat
                vel = new Vector3(_rb.velocity.x, 0f, _rb.velocity.z);
            }

            float speed = vel.magnitude;
            if (speed <= _data.MoveSpeed) return;

            Vector3 restraintSpeed = vel.normalized * _data.MoveSpeed;
            if(!_isOnSlope) {
                restraintSpeed += Vector3.up * _rb.velocity.y;
            }

            _rb.velocity = restraintSpeed;
        }

        private bool OnSlope() {
            if (!Physics.Raycast(_groundCheck.position, Vector3.down, out _slopeHit, _data.SlopeCastDistance, _data.WhatIsGround))
                return false;

            float angle = Vector3.Angle(Vector3.up, _slopeHit.normal);
            return angle < _data.MaxSlopeAngle && angle != 0;
        }

        private Vector3 GetSlopeMoveDirection() {
            return Vector3.ProjectOnPlane(_moveDir, _slopeHit.normal);
        }

        private void OnDrawGizmosSelected() {
            if (!_groundCheck) return;
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(_groundCheck.position, _groundCheck.position + Vector3.down * _data.SlopeCastDistance);

            Gizmos.color = Color.green;
            Gizmos.DrawLine(_groundCheck.position, _groundCheck.position + Vector3.down * _data.GroundCheckDistance);
        }
    }
}
