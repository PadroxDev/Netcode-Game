using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Padrox
{
    public class ExposedScriptableObjectAttribute : PropertyAttribute { }

    [CustomPropertyDrawer(typeof(ExposedScriptableObjectAttribute))]
    public class ExposedScriptableObjectAttributeDrawer : PropertyDrawer {
        private Editor _editor = null;

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
            EditorGUI.PropertyField(position, property, label, true);

            if(property.objectReferenceValue != null) {
                property.isExpanded = EditorGUI.Foldout(position, property.isExpanded, GUIContent.none);
            }

            if(property.isExpanded) {
                EditorGUI.indentLevel++;

                if (!_editor)
                    Editor.CreateCachedEditor(property.objectReferenceValue, null, ref _editor);
                _editor.OnInspectorGUI();

                EditorGUI.indentLevel--;
            }
        }
    }
}
