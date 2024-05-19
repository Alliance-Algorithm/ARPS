using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Linq;
using System;
using System.Net.Security;
using UnityEngine.UI;
using System.IO;
using Unity.VisualScripting;


namespace Radar
{
    public class RayCast : MonoBehaviour
    {
        public Rigidbody cameraObject;
        public Camera radarCamera;

        public float CameraHeight = 2160f;
        public float CameraWidth = 3840f;
        ROSConnection rosConnection_receive;
        ROSConnection rosConnection_send;
        Float32MultiArrayMsg car_msgs;
        List<float> car_positions;
        private bool IsCalibrate = true;


        string receive_topic = "detect_result";

        string submit_topic = "car_positions";
        // Start is called before the first frame update
        void Start()
        {
            car_positions = new List<float>();

            rosConnection_receive = ROSConnection.GetOrCreateInstance();
            rosConnection_receive.Subscribe<Float32MultiArrayMsg>(receive_topic, PositionRaycast);

            rosConnection_send = ROSConnection.GetOrCreateInstance();
            rosConnection_send.RegisterPublisher<Float32MultiArrayMsg>(submit_topic);

            cameraObject = GetComponent<Rigidbody>();
            radarCamera = cameraObject.GetComponentInChildren<Camera>();

            radarCamera.pixelRect = new Rect(0, 0, 3840, 2160);
            readCameraData();

        }
        void PositionRaycast(Float32MultiArrayMsg car_positions_camera)
        {
            car_positions.Clear();
            for (int i = 0; i < car_positions_camera.data.Length - 2; i += 3)
            {
                float classid, x, y;

                classid = car_positions_camera.data[i];
                x = car_positions_camera.data[i + 1] * (radarCamera.pixelWidth / CameraWidth);
                y = radarCamera.pixelHeight - car_positions_camera.data[i + 2] * (radarCamera.pixelHeight / CameraHeight);

                Vector2 car_position_2d = new Vector2(x, y);
                Ray ray = radarCamera.ScreenPointToRay(car_position_2d);
                RaycastHit hit;
                bool isCollider = Physics.Raycast(ray, out hit);
                if (isCollider)
                {

                    Debug.DrawLine(cameraObject.transform.position, hit.point, Color.green, 1.0f);
                    Vector3 hitPointUp;
                    hitPointUp = hit.point;
                    hitPointUp.y += 1.0f;
                    Debug.DrawLine(hitPointUp, hit.point, Color.red, 1.0f);

                    car_positions.Add(classid);
                    car_positions.Add(hit.point.x + 14.0f);
                    car_positions.Add(hit.point.z + 7.5f);

                    // Console.WriteLine("==========");
                    // Console.WriteLine(classid);
                    // Console.WriteLine(hit.point.x + 14.0f);
                    // Console.WriteLine(hit.point.z + 7.5f);
                }
            }
            if (car_positions.Count > 0)
            {
                SendMsg(car_positions);
            }
        }

        public void SendMsg(List<float> car_positions)
        {
            car_msgs = new Float32MultiArrayMsg(
                new MultiArrayLayoutMsg(),
                car_positions.ToArray()
            );
            rosConnection_send.Publish(submit_topic, car_msgs);
        }

        // Update is called once per frame
        void Update()
        {
            if (!IsCalibrate)
            {
                cameraObject.transform.localEulerAngles += new Vector3(0, Input.GetAxis("Mouse X") * 0.1f, 0);
                cameraObject.transform.localEulerAngles += new Vector3(-Input.GetAxis("Mouse Y") * 0.1f, 0, 0);

                if (Input.GetKey(KeyCode.W))
                {
                    Vector3 move_vector = new Vector3(-0.005f, 0, 0);
                    cameraObject.transform.localPosition += move_vector;
                }
                if (Input.GetKey(KeyCode.A))
                {
                    Vector3 move_vector = new Vector3(0, 0, -0.005f);
                    cameraObject.transform.localPosition += move_vector;
                }
                if (Input.GetKey(KeyCode.S))
                {
                    Vector3 move_vector = new Vector3(0.005f, 0, 0);
                    cameraObject.transform.localPosition += move_vector;
                }
                if (Input.GetKey(KeyCode.D))
                {
                    Vector3 move_vector = new Vector3(0, 0, 0.005f);
                    cameraObject.transform.localPosition += move_vector;
                }
                if (Input.GetKey(KeyCode.LeftShift))
                {
                    Vector3 move_vector = new Vector3(0, 0.005f, 0);
                    cameraObject.transform.localPosition += move_vector;
                }
                if (Input.GetKey(KeyCode.LeftControl))
                {
                    Vector3 move_vector = new Vector3(0, -0.005f, 0);
                    cameraObject.transform.localPosition += move_vector;
                }
                if (Input.GetKey(KeyCode.UpArrow))
                {
                    Vector3 rotate_vector = new Vector3(-0.07f, 0, 0);
                    cameraObject.transform.localEulerAngles += rotate_vector;
                }
                if (Input.GetKey(KeyCode.DownArrow))
                {
                    Vector3 rotate_vector = new Vector3(0.07f, 0, 0);
                    cameraObject.transform.localEulerAngles += rotate_vector;
                }
                if (Input.GetKey(KeyCode.LeftArrow))
                {
                    Vector3 rotate_vector = new Vector3(0, -0.07f, 0);
                    cameraObject.transform.localEulerAngles += rotate_vector;
                }
                if (Input.GetKey(KeyCode.RightArrow))
                {
                    Vector3 rotate_vector = new Vector3(0, 0.07f, 0);
                    cameraObject.transform.localEulerAngles += rotate_vector;
                }
                if (Input.GetKey(KeyCode.Q))
                {
                    radarCamera.focalLength += 0.1f;
                }
                if (Input.GetKey(KeyCode.E))
                {
                    radarCamera.focalLength -= 0.1f;
                }
                if (Input.GetKey(KeyCode.P))
                {
                    writeCameraData();
                    IsCalibrate = true;
                }
            }
            if (Input.GetKey(KeyCode.Tab))
            {
                readCameraData();
                IsCalibrate = false;
            }
        }
        void readCameraData()
        {
            string readData;
            //获取到路径
            string fileUrl = Application.absoluteURL + "CameraData.json";
            //读取文件
            if (File.Exists(fileUrl))
            {
                using (StreamReader sr = File.OpenText(fileUrl))
                {

                    readData = sr.ReadToEnd();
                    sr.Close();
                }
                if (readData.Length > 0)
                {
                    CameraData cameraData = JsonUtility.FromJson<CameraData>(readData);
                    Vector3 cameraPositionVector = new Vector3(cameraData.CameraPositionX, cameraData.CameraPositionY, cameraData.CameraPositionZ);
                    Vector3 cameraEulerAngleVector = new Vector3(cameraData.CameraEulerAngleX, cameraData.CameraEulerAngleY, cameraData.CameraEulerAngleZ);
                    cameraObject.transform.localPosition = cameraPositionVector;
                    cameraObject.transform.localEulerAngles = cameraEulerAngleVector;
                    radarCamera.focalLength = cameraData.CameraFocalLength;
                    Console.WriteLine("-----\nRead Camera Data:\n");
                    Console.WriteLine("Camera Position:" + cameraPositionVector + "\n");
                    Console.WriteLine("Camera Angle:" + cameraEulerAngleVector + "\n");
                    Console.WriteLine("Camera FocalLength:" + cameraData.CameraFocalLength + "\n");
                    Console.WriteLine("-----\n");
                }
            }
            else
            {
                Console.WriteLine("Data is empty");
            }

        }

        void writeCameraData()
        {
            CameraData cameraData = new CameraData();

            Vector3 cameraPositionVector = cameraObject.transform.localPosition;
            Vector3 cameraEulerAngleVector = cameraObject.transform.localEulerAngles;
            cameraData.CameraPositionX = cameraPositionVector.x;
            cameraData.CameraPositionY = cameraPositionVector.y;
            cameraData.CameraPositionZ = cameraPositionVector.z;

            cameraData.CameraEulerAngleX = cameraEulerAngleVector.x;
            cameraData.CameraEulerAngleY = cameraEulerAngleVector.y;
            cameraData.CameraEulerAngleZ = cameraEulerAngleVector.z;

            cameraData.CameraFocalLength = radarCamera.focalLength;

            string js = JsonUtility.ToJson(cameraData);
            string fileUrl = Application.absoluteURL + "CameraData.json";
            using (StreamWriter sw = new StreamWriter(fileUrl))
            {
                //保存数据
                sw.WriteLine(js);
                //关闭文档
                sw.Close();
                sw.Dispose();
            }
            Console.WriteLine("Write Done");
        }

    }
}