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
    // Hand objects
    // public OVRSkeleton RightHandSkeleton;
    public OVRPassthroughLayer PassthroughLayerManager;
    private List<OVRBone> RightHandFingerBones;

    // Menu and RayCaster GameObjects
    public GameObject MenuButton;
    public GameObject ResolutionButton;
    public GameObject HighResolutionButton;
    public GameObject LowResolutionButton;

    public GameObject WristTracker;
    private GameObject LaserPointer;
    private LineRenderer LineRenderer;


    // Hand Usage indicator
    public RawImage StreamBorder;

    // Stream Enablers
    bool StreamRelativeData = true;
    bool StreamAbsoluteData = false;


    //Newly added
    //bool HighResolution = false;
    //bool LowResolution = false;
    bool StreamResolution= true;
    public HighResolutionButtonController HighResolutionButtonController;
    public LowResolutionButtonController LowResolutionButtonController;




    // Network enablers
    private NetworkManager netConfig;
    private PushSocket client3;

    private string PauseAddress;

    private string pauseState;

    private bool ShouldContinueArmTeleop = false;

    private bool PauseEstablished = false;
    private bool PauseCreated = false;

    // Controller tracking
    private PushSocket RemoteClient;
    private string RemoteCommunicationAddress;
    private bool RemoteConnectionEstablished = false;

    // Mode
    private int currentMode = 0;

    // Starting the server connection
    public void CreateRemoteTCPConnection()
    {
        // Check if communication address is available
        RemoteCommunicationAddress = netConfig.getRemoteAddress();
        bool AddressAvailable = !String.Equals(RemoteCommunicationAddress, "tcp://:");

        if (AddressAvailable)
        {
            // Initiate Push Socket
            RemoteClient = new PushSocket();
            RemoteClient.Connect(RemoteCommunicationAddress);
            RemoteConnectionEstablished = true;
            RemoteClient.SendFrame("Connected");
        }

        // Setting color to green to indicate control
        if (RemoteConnectionEstablished)
        {
            StreamBorder.color = Color.green;
            ToggleMenuButton(false);
        } else
        {
            StreamBorder.color = Color.red;
            ToggleMenuButton(true);
        }
    }
  

    public void ToggleMenuButton(bool toggle)
    {
        MenuButton.SetActive(toggle);
        LineRenderer.enabled = toggle;
    }

    public void ToggleResolutionButton(bool toggle)
    {
        ResolutionButton.SetActive(toggle);
        LineRenderer.enabled = toggle;
    }

    public void ToggleHighResolutionButton(bool toggle)
    {
        HighResolutionButton.SetActive(toggle);
        
    }

    public void ToggleLowResolutionButton(bool toggle)
    {
        LowResolutionButton.SetActive(toggle);


    }
    // Start function
    void Start()
     {
        // Getting the Network Config Updater gameobject
        GameObject netConfGameObject = GameObject.Find("NetworkConfigsLoader");
        netConfig = netConfGameObject.GetComponent<NetworkManager>();

        LaserPointer = GameObject.Find("LaserPointer");
        LineRenderer = LaserPointer.GetComponent<LineRenderer>();       
        
    }


    public void SendRemoteData(String TypeMarker)
    {
        // Message needs to contain Marker|x,y,z|ax,ay,az,aw|gripper
        string message = TypeMarker + "|";
        Vector3 pos = OVRInput.GetLocalControllerPosition(RightController);
        Quaternion quat = OVRInput.GetLocalControllerRotation(RightController);
        
        message = message + pos.x + "," + pos.y + "," + pos.z + "|";
        message = message + quat.w + "," + quat.x + "," + quat.y + "," + quat.z + "|";
        message = message + OVRInput.Get(OVRInput.RawButton.RIndexTrigger);

        RemoteClient.SendFrame(message);
        byte[] recievedToken = RemoteClient.ReceiveFrameBytes();
    }


    public void SendResetStatus()
    {
        PauseAddress = netConfig.getPauseAddress();
        bool PauseAvailable = !String.Equals(PauseAddress, "tcp://:");

        if (PauseAvailable)
        {   if (!PauseCreated)
            // Initiate Push Socket
            { 
                Debug.Log("Address Available");
                client3 = new PushSocket();
                client3.Connect(PauseAddress);
                PauseEstablished = true;
                PauseCreated=true;
            }
            else 
            {
                PauseEstablished=true;
            }
        }
        else
        {
            PauseEstablished = false;
        }
        
        if (PauseEstablished)
        {
            if (ShouldContinueArmTeleop){
                pauseState = "High";
            } else {
                pauseState = "Low";
            }
            client3.SendFrame(pauseState);
        }
        else 
        {
            
            pauseState="None";
            client3.SendFrame(pauseState);
        }
        

    }
    

    public void StreamPauser()
    {
        // Switching from Right hand control
        bool modeChange = false;
        if (OVRInput.GetDown(OVRInput.RawButton.B))
        {
            currentMode += 1;
            currentMode %= 2;
            modeChange = true;
        }

        if (modeChange && currentMode == 1)
        {
            StreamRelativeData = false;
            StreamAbsoluteData = true;
            StreamResolution= false;
            StreamBorder.color = Color.blue; // Blue for left hand stream
            ToggleMenuButton(false);
            ToggleResolutionButton(false);
            WristTracker.SetActive(true);
            ShouldContinueArmTeleop = true;
            // SendCont();
            
        }

        // // Switching from Left hand control
        // if (OVRInput.Get(OVRInput.RawButton.Y))
        // {
        //     StreamRelativeData = true;
        //     StreamAbsoluteData = false;
        //     StreamResolution = false;
        //     StreamBorder.color = Color.green; // Green for right hand stream
        //     ToggleMenuButton(false);
        //     ToggleResolutionButton(false);
        //     WristTracker.SetActive(false);
        //     ShouldContinueArmTeleop = false;
        //     // SendPause();
            
        // }

        // Pausing Stream
        if (modeChange && currentMode == 0)
        {
            StreamRelativeData = false;
            StreamAbsoluteData = false;
            StreamResolution = false;
            StreamBorder.color = Color.red; // Red color for no stream 
            ToggleMenuButton(true);
            //ToggleResolutionButton(false);
            WristTracker.SetActive(false);
            ShouldContinueArmTeleop = false;
            
        }

        // if (LeftHand.GetFingerIsPinching(OVRHand.HandFinger.Pinky))
        // {
        //     StreamRelativeData = false;
        //     StreamAbsoluteData = false;
        //     StreamResolution = true;
        //     StreamBorder.color = Color.black; // Black color for resolution
        //     ShouldContinueArmTeleop = false;
        //     ToggleMenuButton(false);
        //     ToggleResolutionButton(true);
        //     WristTracker.SetActive(false);
        //     // SendPause();

        // }
    }

    void Update()
    {
        // Hand Tracking
        if (RemoteConnectionEstablished)
        
        {   
            SendResetStatus();
            if (String.Equals(RemoteCommunicationAddress, netConfig.getRemoteAddress()))
            {   

                StreamPauser();

                

                if (StreamAbsoluteData)
                {   
                    SendRemoteData("absolute");
                    ToggleResolutionButton(false);
                    // SendCont();
                   
                }

                if (StreamRelativeData)
                {
                    SendRemoteData("relative");
                    ToggleResolutionButton(false);
                    // SendPause();
                    
                }

                if (StreamResolution)
                {   
                    ToggleHighResolutionButton(true);
                    ToggleLowResolutionButton(true);
                    // SendPause();
                   
                } 
              
            }
            else
            {
                RemoteConnectionEstablished = false;
            }
        
        } else
        {
            StreamBorder.color = Color.red;
            ToggleMenuButton(true);
            //ToggleResolutionButton(false);
            CreateRemoteTCPConnection();        
        }
    }
}