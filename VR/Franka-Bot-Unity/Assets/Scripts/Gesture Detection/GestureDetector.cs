using System;
using System.Collections.Generic;

using UnityEngine;
using UnityEngine.UI;

using NetMQ;
using NetMQ.Sockets;

class GestureDetector : MonoBehaviour
{
    // Controller Objects
    public OVRInput.Controller RightController;
    [SerializeField] private ChangeColor changeColorScript;
    private Color controllerColor = Color.red; // Default color for the controller mesh
    public Transform controllerTransform;

    // Menu and RayCaster GameObjects
    public GameObject MenuButton;
    public GameObject WristTracker;
    private GameObject LaserPointer;
    private LineRenderer LineRenderer;

    // Stream Enablers
    bool StreamAbsoluteData = false;

    // Network enablers
    private NetworkManager netConfig;

    // Pause Socket
    private PushSocket PauseClient;
    private string pauseState;
    private bool ShouldContinueArmTeleop = false;
    private bool PauseConnectionEstablished = false;

    // Controller Socket
    private PushSocket RemoteClient;
    private string RemoteCommunicationAddress;
    private bool RemoteConnectionEstablished = false;

    // Mode
    private int currentMode = 0;

    // Start function
    void Start()
    {
        AsyncIO.ForceDotNet.Force();
        NetMQConfig.Cleanup(false);
        GameObject netConfGameObject = GameObject.Find("NetworkConfigsLoader");

        if (netConfGameObject == null)
        {
            Debug.LogError("NetworkConfigsLoader not found in the scene!");
            return;  // Prevent further execution
        }

        netConfig = netConfGameObject.GetComponent<NetworkManager>();

        if (netConfig == null)
        {
            Debug.LogError("NetworkManager component not found on NetworkConfigsLoader!");
            return;  // Prevent further execution
        }

        LaserPointer = GameObject.Find("LaserPointer");
        LineRenderer = LaserPointer.GetComponent<LineRenderer>();

        // Initiate Push Socket
        RemoteClient = new PushSocket();
        ConnectRemoteClient(netConfig.getRemoteAddress());

        // Initiate Push Socket for Pause
        PauseClient = new PushSocket();
        ConnectPauseClient(netConfig.getPauseAddress());
    }

    // Starting the server connection
    public void ConnectRemoteClient(string address)
    {
        // Check if communication address is available
        bool AddressAvailable = !String.Equals(address, "tcp://:");

        if (AddressAvailable)
        {
            RemoteCommunicationAddress = address;
            RemoteClient.Connect(address);
            RemoteConnectionEstablished = true;
            Debug.Log("Connected to Remote Client at: " + address);
        }

        // Setting color to green to indicate control
        if (RemoteConnectionEstablished)
        {
            controllerColor = Color.red; // Start in red since were not streaming the controller
            ToggleMenuButton(false);
        }
        else
        {
            controllerColor = Color.magenta; // Set the controller color to magenta to indicate an issue
            ToggleMenuButton(true);
            Debug.LogWarning("Remote Client connection failed or address is not set correctly: " + address);
        }
    }

    public void ConnectPauseClient(string address)
    {
        // Check if communication address is available
        bool PauseAvailable = !String.Equals(address, "tcp://:");

        if (PauseAvailable)
        {
            PauseClient.Connect(address);
            PauseConnectionEstablished = true;
        }
    }


    public void ToggleMenuButton(bool toggle)
    {
        MenuButton.SetActive(toggle);
        LineRenderer.enabled = toggle;

    }


    public void SendRemoteData(String TypeMarker)
    {
        // Message needs to contain Marker|x,y,z|q1,q2,q3,q4|gripper|offset_forward,offset_right,offset_up
        Vector3 pos = OVRInput.GetLocalControllerPosition(RightController);
        Quaternion quat = OVRInput.GetLocalControllerRotation(RightController);
        Vector3 offsetForward = pos + controllerTransform.forward * 0.1f;
        Vector3 offsetRight = pos + controllerTransform.right * 0.1f;
        Vector3 offsetUp = pos + controllerTransform.up * 0.1f;


        string message = TypeMarker + "|";
        message = message + pos.x + "," + pos.y + "," + pos.z + "|";
        message = message + quat.w + "," + quat.x + "," + quat.y + "," + quat.z + "|";
        message = message + OVRInput.Get(OVRInput.RawButton.RIndexTrigger) + "|";
        message = message + offsetForward.x + "," + offsetForward.y + "," + offsetForward.z + "|";
        message = message + offsetRight.x + "," + offsetRight.y + "," + offsetRight.z + "|";
        message = message + offsetUp.x + "," + offsetUp.y + "," + offsetUp.z;

        bool sent = RemoteClient.TrySendFrame(message);
        if (!sent) {
            Debug.LogWarning("Message send failed or would block");
        }
    }


    public void SendResetStatus()
    {
        if (PauseConnectionEstablished)
        {
            if (ShouldContinueArmTeleop)
            {
                pauseState = "High";
            }
            else
            {
                pauseState = "Low";
            }
            bool sent = PauseClient.TrySendFrame(pauseState);
            if (!sent) {
                Debug.LogWarning("Message send failed or would block");
            }
        }
    }


    public void StreamPauser()
    {
        // Check if B is pressed only on first frame down so hold does not toggle the mode
        bool modeChange = false;
        if (OVRInput.GetDown(OVRInput.RawButton.B))
        {
            currentMode += 1;
            currentMode %= 2;
            modeChange = true;
        }

        // Streams Data
        if (modeChange && currentMode == 1)
        {
            OVRManager.display.RecenterPose();
            StreamAbsoluteData = true;
            controllerColor = Color.green; // Set the controller color to green
            ToggleMenuButton(false);
            WristTracker.SetActive(true);
            ShouldContinueArmTeleop = true;

        }

        // Pausing Stream
        if (modeChange && currentMode == 0)  // open menu
        {
            StreamAbsoluteData = false;
            controllerColor = Color.red; // Set the controller color to red
            ToggleMenuButton(true);
            WristTracker.SetActive(false);
            ShouldContinueArmTeleop = false;

        }

    }

    void Update()
    {
        changeColorScript.SetColor(controllerColor);  // Update controller color
        // Hand Tracking
        if (RemoteConnectionEstablished)

        {
            SendResetStatus();
            if (String.Equals(RemoteCommunicationAddress, netConfig.getRemoteAddress()) && !String.Equals(RemoteCommunicationAddress, "tcp://:"))
            {

                StreamPauser();  // check if B was pressed to bring up the menu

                // Stream Data
                if (StreamAbsoluteData)
                    SendRemoteData("absolute");
                
            }
            else
            {
                RemoteConnectionEstablished = false;
            }

        }
        else
        {
            Debug.LogWarning("Update function: connection not established");
            ToggleMenuButton(true);
            ConnectRemoteClient(netConfig.getRemoteAddress());
            ConnectPauseClient(netConfig.getPauseAddress());
        }
    }

    void OnApplicationQuit()
    {
        // Clean up NetMQ resources
        PauseClient.Dispose();
        RemoteClient.Dispose();
        NetMQConfig.Cleanup(false);
    }
}