using System.Collections;
using System.Collections.Generic;
using Unity.Netcode;
using UnityEngine;

namespace Padrox
{
    public class PlayerController : NetworkBehaviour
    {
        private Rigidbody _rb;

        private void Awake() {
            _rb = GetComponent<Rigidbody>();
        }

        public override void OnNetworkSpawn() {
            if (!IsOwner) Destroy(this);
        }

        private void Update() {
            
        }

        private void FixedUpdate() {
            
        }
    }
}
