using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Padrox
{
    [CreateAssetMenu(menuName = "Player/Data", fileName = "New Player Data")]
    public class PlayerData : ScriptableObject
    {
        [Header("Movement")]
        public float MoveSpeed;
        public float GroundDrag;
        public float AirDrag;

        [Header("Rotation")]
        public float RotationSpeed;

        [Header("Ground Check")]
        public float GroundCheckDistance;
        public float GroundCheckRadius;
        public LayerMask WhatIsGround;
    }
}
