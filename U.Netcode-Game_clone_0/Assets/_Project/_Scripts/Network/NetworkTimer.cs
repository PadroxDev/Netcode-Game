using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Padrox {
    public class NetworkTimer {
        private float _timer;

        public int CurrentTick { get; private set; }
        public float MinTimeBetweenTicks { get; }

        public NetworkTimer(float serverTickRate) {
            MinTimeBetweenTicks = 1 / serverTickRate;
        }

        public void Update(float deltaTime) {
            _timer += deltaTime;
        }

        public bool ShouldTick() {
            if (_timer < MinTimeBetweenTicks) return false;

            _timer -= MinTimeBetweenTicks;
            CurrentTick++;
            return true;
        }
    }
}
