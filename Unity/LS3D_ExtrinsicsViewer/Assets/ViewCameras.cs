using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

namespace CamView
{
    public class ViewCameras : MonoBehaviour
    {
        [SerializeField] private string filepath = "Assets/Resources/TextAssets";
        [SerializeField] private string filename = "Extrinsics_Open3D.log";

        [SerializeField] private Transform cameraPrefab = null;

        private List<string> extrinsicsStrings = new List<string>();
        [SerializeField] private List<Matrix4x4> cameraMatrices = new List<Matrix4x4>();
        [SerializeField] private List<Transform> cameras = new List<Transform>();

        private int cameraCount = 0;

        private void Awake()
        {
            ParseTextFile();
        }

        private void ParseTextFile()
        {
            StreamReader reader = new StreamReader(filepath + "/" + filename);

            extrinsicsStrings.Clear();
            cameraMatrices.Clear();

            while (!reader.EndOfStream)
            {
                extrinsicsStrings.Add(reader.ReadLine());
            }

            Matrix4x4 parsingMatrix = Matrix4x4.identity;
            float matrixValue = 0.0f;
            cameraCount = 0;

            for (int i = 0; i < extrinsicsStrings.Count; i += 5, ++cameraCount)
            {
                for (int j = 0; j < 4; ++j)
                {
                    int k = 0;

                    foreach (var s in extrinsicsStrings[j + i + 1].Split('\t'))
                    {
                        if (float.TryParse(s, out matrixValue))
                        {
                            parsingMatrix[j, k] = matrixValue;
                        }

                        ++k;
                    }
                }

                cameraMatrices.Add(parsingMatrix);

                cameras.Add(Instantiate(cameraPrefab, parsingMatrix.ExtractPosition(), parsingMatrix.ExtractRotation()));
            }
        }
    }

    public static class MatrixExtensions
    {
        public static Quaternion ExtractRotation(this Matrix4x4 matrix)
        {
            Vector3 forward;
            forward.x = matrix.m02;
            forward.y = matrix.m12;
            forward.z = matrix.m22;

            Vector3 upwards;
            upwards.x = matrix.m01;
            upwards.y = matrix.m11;
            upwards.z = matrix.m21;

            return Quaternion.LookRotation(forward, upwards);
        }

        public static Vector3 ExtractPosition(this Matrix4x4 matrix)
        {
            Vector3 position;
            position.x = matrix.m03;
            position.y = matrix.m13;
            position.z = matrix.m23;
            return position;
        }

        public static Vector3 ExtractScale(this Matrix4x4 matrix)
        {
            Vector3 scale;
            scale.x = new Vector4(matrix.m00, matrix.m10, matrix.m20, matrix.m30).magnitude;
            scale.y = new Vector4(matrix.m01, matrix.m11, matrix.m21, matrix.m31).magnitude;
            scale.z = new Vector4(matrix.m02, matrix.m12, matrix.m22, matrix.m32).magnitude;
            return scale;
        }
    }
}