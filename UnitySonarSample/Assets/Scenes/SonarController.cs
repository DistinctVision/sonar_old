using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using UnityEngine.UI;

public class SonarController : MonoBehaviour
{
    public enum TrackingStateType : int
    {
        NotStarted = 0,
        Initializing,
        Tracking,
        Lost
    };

    WebCamTexture webCamTexture;
    public RawImage targetCameraImage;
    public GameObject targetObject;
    public Quaternion sonar_cameraRotation;
    public Vector3 sonar_cameraPosition;

    bool pauseFlag = false;

    Vector2[][] debugInitPoints = new Vector2[3][];

    public TrackingStateType TrackingState
    {
        get { return (TrackingStateType)sonar_getTrackingState(); }
    }
    
    void Start()
    {
        sonar_reset();
        webCamTexture = new WebCamTexture(WebCamTexture.devices[1].name, 640, 480);
        webCamTexture.Play();
        targetCameraImage.texture = webCamTexture;
        targetObject.SetActive(false);

        sonar_setPinholeCamera(webCamTexture.width, webCamTexture.height,
                              - (double)webCamTexture.width, - (double)webCamTexture.width,
                              webCamTexture.width * 0.5, webCamTexture.height * 0.5);
    }

    void Update()
    {
        if (webCamTexture.isPlaying)
        {
            Color32[] colors = webCamTexture.GetPixels32();
            byte[] bw_colors = new byte[colors.Length];
            for (int i = 0; i < colors.Length; ++i)
            {
                bw_colors[i] = (byte)((colors[i].r + colors[i].g + colors[i].b) / 3);
            }
            unsafe
            {
                fixed (byte* ptr = &bw_colors[0])
                {
                    sonar_process_image_frame((System.IntPtr)ptr, webCamTexture.width, webCamTexture.height);
                }
            }
        }
        switch (TrackingState)
        {
            case TrackingStateType.NotStarted:
                targetObject.SetActive(false);
                if (Input.GetKeyUp(KeyCode.Space))
                {
                    sonar_start();
                }
                break;
            case TrackingStateType.Initializing:
                targetObject.SetActive(false);
                for (int i = 0; i < 3; ++i)
                {
                    debugInitPoints[i] = new Vector2[sonar_get_number_init_image_points(i)];
                    if (debugInitPoints[i].Length > 0)
                    {
                        unsafe
                        {
                            fixed (Vector2* ptr = &debugInitPoints[i][0])
                            {
                                sonar_get_init_image_points((System.IntPtr)ptr, i);
                            }
                        }
                    }
                }
                break;
            case TrackingStateType.Tracking:
                UpdatePose();
                if (Input.GetKeyUp(KeyCode.Space) || Input.GetKeyUp(KeyCode.Escape))
                {
                    sonar_reset();
                    webCamTexture.Play();
                }
                else
                {
                    webCamTexture.Stop();
                }
                break;
            default:
                targetObject.SetActive(false);
                break;
        }
    }

    public Material lineMat = null;
    void OnPostRender()
    {
        if (TrackingState == TrackingStateType.Initializing)
        {
            GL.Clear(true, false, Color.black);
            GL.PushMatrix();
            lineMat.SetPass(0);
            GL.LoadOrtho();
            GL.Begin(GL.LINES);

            Vector2 frameSize = new Vector2((float)webCamTexture.width, (float)webCamTexture.height);

            for (int i = 0; i < 2; ++i)
            {
                Vector2[] pointsA = debugInitPoints[i];
                Vector2[] pointsB = debugInitPoints[i+1];
                if ((pointsA == null) || (pointsB == null))
                {
                    continue;
                }
                if ((pointsA.Length == 0) || (pointsB.Length == 0))
                {
                    continue;
                }
                int size = Mathf.Min(pointsA.Length, pointsB.Length);
                for (int j = 0; j < size; ++j)
                {
                    Vector2 pA = pointsA[j];
                    Vector2 pB = pointsB[j];
                    GL.Vertex3(pA.x / frameSize.x, pA.y / frameSize.y, 0f);
                    GL.Vertex3(pB.x / frameSize.x, pB.y / frameSize.y, 0f);
                }
            }

            GL.End();
            GL.PopMatrix();
        }
    }

    void UpdatePose()
    {
        double[] q_data = new double[4];
        double[] t_data = new double[3];
        unsafe
        {
            fixed (double* q_ptr = &q_data[0])
            {
                fixed (double* t_ptr = &t_data[0])
                {
                    sonar_get_current_frame_pose_qt((System.IntPtr)q_ptr, (System.IntPtr)t_ptr);
                }
            }
        }
        Quaternion q = new Quaternion(- (float)q_data[0], - (float)q_data[1], - (float)q_data[2], (float)q_data[3]);
        Vector3 t = q * new Vector3((float)t_data[0], (float)t_data[1], (float)t_data[2]);

        transform.rotation = q;
        transform.position = t;
    }

    [DllImport("sonar")]
    private static extern void sonar_setPinholeCamera(int imageWidth, int imageHeight,
                                                      double focalLength_x, double focalLength_y,
                                                      double opticalCenter_x, double opticalCenter_y);

    [DllImport("sonar")]
    private static extern int sonar_getTrackingState();

    [DllImport("sonar")]
    private static extern void sonar_start();

    [DllImport("sonar")]
    private static extern void sonar_reset();

    [DllImport("sonar")]
    private static extern void sonar_process_image_frame(System.IntPtr imageData, int imageWidth, int imageHeight);

    [DllImport("sonar")]
    private static extern int sonar_get_current_frame_pose_Rt(System.IntPtr out_rotation_ptr, System.IntPtr out_translation_ptr);

    [DllImport("sonar")]
    private static extern int sonar_get_current_frame_pose_qt(System.IntPtr out_quaternion_ptr, System.IntPtr out_translation_ptr);

    [DllImport("sonar")]
    private static extern int sonar_get_number_init_image_points(int indexStep);

    [DllImport("sonar")]
    private static extern void sonar_get_init_image_points(System.IntPtr out_data_points, int indexStep);
}
