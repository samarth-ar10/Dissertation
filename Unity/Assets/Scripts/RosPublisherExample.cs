using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Assuming you have a message type for joint states

public class UR3JointTrajectoryPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_input";

    // UR3 robot links
    public GameObject ur3_link1;
    public GameObject ur3_link2;
    public GameObject ur3_link3;
    public GameObject ur3_link4;
    public GameObject ur3_link5;
    public GameObject ur3_link6;

    // Publish frequency
    public float publishMessageFrequency = 0.5f;
    private float timeElapsed;

    void Start()
    {
        // Start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float64MultiArrayMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // Create a message with the joint positions from UR_Stream_Data.J_Orientation
            Float64MultiArrayMsg jointTrajectoryMsg = new Float64MultiArrayMsg
            {
                data = new double[]
                {
                    ur_data_processing.UR_Stream_Data.J_Orientation[0],
                    ur_data_processing.UR_Stream_Data.J_Orientation[1],
                    ur_data_processing.UR_Stream_Data.J_Orientation[2],
                    ur_data_processing.UR_Stream_Data.J_Orientation[3],
                    ur_data_processing.UR_Stream_Data.J_Orientation[4],
                    ur_data_processing.UR_Stream_Data.J_Orientation[5]
                }
            };

            // Publish the message
            ros.Publish(topicName, jointTrajectoryMsg);

            timeElapsed = 0;
        }
    }
}