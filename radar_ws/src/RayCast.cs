using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Linq;
using System;
using System.Net.Security;
using UnityEngine.UI;



public class RayCast : MonoBehaviour
{
    public Rigidbody cameraObject;
    public Camera radarCamera;
    public GameObject PrefabRobot;

    public float CameraHeight = 2160f;
    public float CameraWidth = 3840f;
    ROSConnection rosConnection_receive;
    ROSConnection rosConnection_send;
    Float32MultiArrayMsg car_msgs;
    List<float> car_positions;
    private bool IsCalibrate = true;
    List<GameObject> robots = new();


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

                // Debug.Log("==========");
                // Debug.Log(classid);
                // Debug.Log(hit.point.x + 14.0f);
                // Debug.Log(hit.point.z + 7.5f);
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
                IsCalibrate = true;
            }
        }
        if (Input.GetKey(KeyCode.Tab))
        {
            IsCalibrate = false;
        }
    }
}
