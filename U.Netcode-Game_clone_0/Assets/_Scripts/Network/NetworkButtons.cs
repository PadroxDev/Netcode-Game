using System.Collections;
using System.Collections.Generic;
using Unity.Netcode;
using UnityEngine;

namespace Padrox
{
    public class NetworkButtons : MonoBehaviour
    {
        private void OnGUI() {
            GUILayout.BeginArea(new Rect(10, 10, 300, 300));
            if(!NetworkManager.Singleton.IsClient && !NetworkManager.Singleton.IsServer) {
                if (GUILayout.Button("Host")) NetworkManager.Singleton.StartHost();
                if (GUILayout.Button("Client")) NetworkManager.Singleton.StartClient();
                if (GUILayout.Button("Server")) NetworkManager.Singleton.StartServer();
            }

            GUILayout.EndArea();
        }
    }
}
