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

    public TrackingStateType TrackingState
    {
        get { return (TrackingStateType)sonar_getTrackingState(); }
    }
    
    void Start()
    {
        webCamTexture = new WebCamTexture(WebCamTexture.devices[0].name, 640, 480);
        targetCameraImage.texture = webCamTexture;
        targetObject.SetActive(false);
    }

    void Update()
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
        switch(TrackingState)
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

                break;
            case TrackingStateType.Tracking:
                if (Input.GetKeyUp(KeyCode.Space) || Input.GetKeyUp(KeyCode.Escape))
                {
                    sonar_reset();
                }
                break;
            default:
                targetObject.SetActive(false);
                break;
        }
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
    private static extern int sonar_get_current_frame_pose(System.IntPtr out_rotation_ptr, System.IntPtr out_translation_ptr);

    [DllImport("sonar")]
    private static extern int sonar_get_number_init_image_points(int indexStep);

    [DllImport("sonar")]
    private static extern void sonar_get_init_image_points(System.IntPtr out_data_points, int indexStep);
}
