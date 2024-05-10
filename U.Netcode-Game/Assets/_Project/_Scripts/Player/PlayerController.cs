using Cinemachine;
using Unity.Netcode;
using UnityEngine;

namespace Padrox
{
    public class PlayerController : NetworkBehaviour
    {
        [ExposedScriptableObject]
        [SerializeField] private PlayerData _data;

        [Space, Header("References")]
        [SerializeField] private Transform _orientation;
        [SerializeField] private Transform _groundCheck;
        [SerializeField] private CinemachineFreeLook _playerCam;
        [SerializeField] private AudioListener _playerAudioListener;

        private Rigidbody _rb;
        private Vector2 _inputDir;
        private Vector3 _moveDir;
        private bool _grounded;

        private void Awake() {
            _rb = GetComponent<Rigidbody>();

            _inputDir = Vector2.zero;
            _moveDir = Vector3.zero;
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
            _inputDir = InputManager.GetMoveDirection();
        }

        private void FixedUpdate() {
            if (!IsOwner) return;

            GroundCheck();
            EvaluateMoveDir();
            HandleMovement();
            HandleRotation();
            HandleDrag();
            SpeedRestraint();
        }

        private void GroundCheck() {
            _grounded = Physics.SphereCast(_groundCheck.position, _data.GroundCheckRadius, Vector3.down,
                out RaycastHit hit, _data.GroundCheckDistance, _data.WhatIsGround);
        }

        private void EvaluateMoveDir() {
            Vector3 camPos = new Vector3(_playerCam.transform.position.x, transform.position.y, _playerCam.transform.position.z);
            Vector3 viewDir = (transform.position - camPos).normalized;
            _orientation.forward = viewDir;

            _moveDir = _orientation.forward * _inputDir.y + _orientation.right * _inputDir.x;
        }

        private void HandleMovement() {
            if (_moveDir == Vector3.zero) return;
            
            _rb.AddForce(_moveDir.normalized * _data.MoveSpeed * 10f, ForceMode.Force);
        }

        private void HandleRotation() {
            if (_moveDir == Vector3.zero) return;
            transform.forward = Vector3.Slerp(transform.forward, _moveDir.normalized, Time.fixedDeltaTime * _data.RotationSpeed);
        }

        private void HandleDrag() {
            _rb.drag = _grounded ? _data.GroundDrag : _data.AirDrag;
        }

        private void SpeedRestraint() {
            Vector3 flatVel = new Vector3(_rb.velocity.x, 0f, _rb.velocity.z);
            float speed = flatVel.magnitude;

            if (speed <= _data.MoveSpeed) return;

            Vector3 restraintSpeed = flatVel.normalized * _data.MoveSpeed;
            _rb.velocity = restraintSpeed + Vector3.up * _rb.velocity.y;
        }

        private void OnDrawGizmosSelected() {
            if (!_groundCheck) return;
            Gizmos.color = Color.green;
            Gizmos.DrawLine(_groundCheck.position, _groundCheck.position + Vector3.down * _data.GroundCheckDistance);
            Gizmos.DrawWireSphere(_groundCheck.position, _data.GroundCheckRadius);
            Gizmos.DrawWireSphere(_groundCheck.position + Vector3.down * _data.GroundCheckDistance * 0.5f, _data.GroundCheckRadius);
            Gizmos.DrawWireSphere(_groundCheck.position + Vector3.down * _data.GroundCheckDistance, _data.GroundCheckRadius);
        }
    }
}
