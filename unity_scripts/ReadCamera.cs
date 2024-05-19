using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using System.Threading;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace Radar
{
    public class ReadCamera : MonoBehaviour
    {
        public RawImage rawImage;//相机渲染的UI
        private WebCamTexture webCamTexture;

        public int cameraSelect = 0;
        public int cameraHeight = 1440;
        public int cameraWidth = 2160;

        void Start()
        {
            rawImage = GetComponentInChildren<RawImage>();
            ToOpenCamera();
        }

        void Update()
        {
            if (Input.GetKey(KeyCode.P))
            {
                OnDestroy();
                StopAllCoroutines();
                rawImage.enabled = false;
            }
            if (Input.GetKey(KeyCode.Tab))
            {
                rawImage.enabled = true;
                ToOpenCamera();
            }
            if (Input.GetKey(KeyCode.K))
            {
                SaveImage();
            }
            if (Input.GetKey(KeyCode.L))
            {
                OnDestroy();
                StopAllCoroutines();
                rawImage.enabled = true;
                readTexture();
            }
        }



        /// <summary>
        /// 打开摄像机
        /// </summary>
        public void ToOpenCamera()
        {
            StartCoroutine(OpenCamera());
        }
        public IEnumerator OpenCamera()
        {
            // 申请摄像头权限
            yield return Application.RequestUserAuthorization(UserAuthorization.WebCam);
            if (Application.HasUserAuthorization(UserAuthorization.WebCam))
            {
                if (webCamTexture != null)
                {
                    webCamTexture.Stop();
                }

                //打开渲染图
                if (rawImage != null)
                {
                    rawImage.gameObject.SetActive(true);
                }

                // 监控第一次授权，是否获得到设备（因为很可能第一次授权了，但是获得不到设备，这里这样避免）
                while (WebCamTexture.devices.Length <= 0)
                {
                    yield return new WaitForEndOfFrame();
                }
                WebCamDevice[] devices = WebCamTexture.devices;//获取可用设备
                                                               // Debug.Log(devices);
                if (WebCamTexture.devices.Length <= 0)
                {
                    Debug.LogError("没有摄像头设备，请检查");
                }
                else
                {
                    string devicename = devices[cameraSelect].name;
                    webCamTexture = new WebCamTexture(devicename, cameraWidth, cameraHeight, 30)
                    {
                        wrapMode = TextureWrapMode.Repeat
                    };


                    // 渲染到 UI 或者 游戏物体上
                    if (rawImage != null)
                    {
                        rawImage.texture = webCamTexture;
                    }
                    webCamTexture.Play();
                    Debug.Log(devices[cameraSelect].name);
                    Debug.Log("webCamTexture.isPlaying:" + webCamTexture.isPlaying);
                    if (!webCamTexture.isPlaying)
                    {
                        cameraSelect = (cameraSelect + 1) < devices.Length ? cameraSelect + 1 : 0;
                        yield return new WaitForSeconds(1);
                        StopAllCoroutines();
                        StartCoroutine(OpenCamera());
                    }
                }

            }
            else
            {
                Debug.LogError("未获得读取摄像头权限");
            }
        }
        private void OnApplicationPause(bool pause)
        {
            // 应用暂停的时候暂停camera，继续的时候继续使用
            if (webCamTexture != null)
            {
                if (pause)
                {
                    webCamTexture.Pause();
                }
                else
                {
                    webCamTexture.Play();
                }
            }

        }
        private void OnDestroy()
        {
            if (webCamTexture != null)
            {
                webCamTexture.Stop();
            }
        }

        public void SaveImage()
        {
            Texture2D texture2D = new Texture2D(webCamTexture.width, webCamTexture.height);
            texture2D.SetPixels(webCamTexture.GetPixels());
            texture2D.Apply();

            // 编码为PNG格式
            byte[] bytes = texture2D.EncodeToPNG();

            // 保存为文件
            string filePath = Path.Combine(Application.absoluteURL, "radar_camera.png");
            File.WriteAllBytes(filePath, bytes);

            Debug.Log("Saved image to: " + filePath);
        }
        public void readTexture()
        {
            string filePath = Path.Combine(Application.absoluteURL, "radar_camera.png");

            if (File.Exists(filePath))
            {
                // 读取PNG文件的字节数据
                byte[] fileData = File.ReadAllBytes(filePath);

                // 创建Texture2D对象
                Texture2D texture2D = new Texture2D(2, 2);
                texture2D.LoadImage(fileData); // 加载字节数据到Texture2D对象中

                // 将Texture2D对象赋值给RawImage的texture属性
                rawImage.texture = texture2D;

                Debug.Log("Loaded image from: " + filePath);
            }
            else
            {
                Debug.LogError("File not found: " + filePath);
            }
        }
    }
}
