using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Netcode;
using UnityEngine;

namespace Padrox
{
    public class PlayerController : NetworkBehaviour
    {
        [SerializeField] private PlayerData _data;

        private Rigidbody _rb;
        private Vector2 _moveDir;

        private void Awake() {
            _rb = GetComponent<Rigidbody>();
            _moveDir = Vector2.zero;
        }

        public override void OnNetworkSpawn() {
            if (!IsOwner) Destroy(this);
        }

        private void Update() {
            _moveDir = InputManager.GetMoveDirection();
        }

        private void FixedUpdate() {
            HandleMovement();
        }

        private void HandleMovement() {
            if(_moveDir == Vector2.zero) return;

            Vector3 dir = new Vector3(_moveDir.x, 0, _moveDir.y);
            Vector3 force = dir * _data.BaseMoveSpeed;
            _rb.AddForce(force);
            transform.forward = dir;
            
            Vector3 vel = _rb.velocity;
            vel.y = 0; // Ignore y
            float speed = vel.magnitude;

            if(speed > _data.BaseMoveSpeed) {
                _rb.velocity = vel * _data.BaseMoveSpeed + Vector3.up * _rb.velocity.y;
            }
        }
    }
}
