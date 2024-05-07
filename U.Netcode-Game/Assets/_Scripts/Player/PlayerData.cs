using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Padrox
{
    [CreateAssetMenu(menuName = "Player/Data", fileName = "New Player Data")]
    public class PlayerData : ScriptableObject
    {
        public float MoveSpeed;
    }
}
