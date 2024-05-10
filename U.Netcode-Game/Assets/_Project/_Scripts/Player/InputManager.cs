using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Padrox
{
    public class InputManager : Singleton<InputManager>
    {
        private static PlayerControls _controls;

        protected override void Awake() {
            base.Awake();
            _controls = new PlayerControls();
        }

        private void OnEnable() {
            _controls.Enable();
        }

        private void OnDisable() {
            _controls.Disable();
        }

        public static Vector2 GetMoveDirection() => _controls.Player.Movement.ReadValue<Vector2>();
    }
}
