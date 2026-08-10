// Generated locally for the piper_ros contract used by this Unity simulation.
using System;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Piper
{
    [Serializable]
    public class PiperStatusMsg : Message
    {
        public const string k_RosMessageName = "piper_msgs/PiperStatusMsg";
        public override string RosMessageName => k_RosMessageName;

        public byte ctrl_mode;
        public byte arm_status;
        public byte mode_feedback;
        public byte teach_status;
        public byte motion_status;
        public byte trajectory_num;
        public long err_code;
        public bool joint_1_angle_limit;
        public bool joint_2_angle_limit;
        public bool joint_3_angle_limit;
        public bool joint_4_angle_limit;
        public bool joint_5_angle_limit;
        public bool joint_6_angle_limit;
        public bool communication_status_joint_1;
        public bool communication_status_joint_2;
        public bool communication_status_joint_3;
        public bool communication_status_joint_4;
        public bool communication_status_joint_5;
        public bool communication_status_joint_6;

        public PiperStatusMsg()
        {
        }

        public static PiperStatusMsg Deserialize(MessageDeserializer deserializer) => new PiperStatusMsg(deserializer);

        private PiperStatusMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out ctrl_mode);
            deserializer.Read(out arm_status);
            deserializer.Read(out mode_feedback);
            deserializer.Read(out teach_status);
            deserializer.Read(out motion_status);
            deserializer.Read(out trajectory_num);
            deserializer.Read(out err_code);
            deserializer.Read(out joint_1_angle_limit);
            deserializer.Read(out joint_2_angle_limit);
            deserializer.Read(out joint_3_angle_limit);
            deserializer.Read(out joint_4_angle_limit);
            deserializer.Read(out joint_5_angle_limit);
            deserializer.Read(out joint_6_angle_limit);
            deserializer.Read(out communication_status_joint_1);
            deserializer.Read(out communication_status_joint_2);
            deserializer.Read(out communication_status_joint_3);
            deserializer.Read(out communication_status_joint_4);
            deserializer.Read(out communication_status_joint_5);
            deserializer.Read(out communication_status_joint_6);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(ctrl_mode);
            serializer.Write(arm_status);
            serializer.Write(mode_feedback);
            serializer.Write(teach_status);
            serializer.Write(motion_status);
            serializer.Write(trajectory_num);
            serializer.Write(err_code);
            serializer.Write(joint_1_angle_limit);
            serializer.Write(joint_2_angle_limit);
            serializer.Write(joint_3_angle_limit);
            serializer.Write(joint_4_angle_limit);
            serializer.Write(joint_5_angle_limit);
            serializer.Write(joint_6_angle_limit);
            serializer.Write(communication_status_joint_1);
            serializer.Write(communication_status_joint_2);
            serializer.Write(communication_status_joint_3);
            serializer.Write(communication_status_joint_4);
            serializer.Write(communication_status_joint_5);
            serializer.Write(communication_status_joint_6);
        }

        public override string ToString()
        {
            var sb = new StringBuilder("PiperStatusMsg: ");
            sb.Append("\nctrl_mode: ").Append(ctrl_mode);
            sb.Append("\narm_status: ").Append(arm_status);
            sb.Append("\nmode_feedback: ").Append(mode_feedback);
            sb.Append("\nteach_status: ").Append(teach_status);
            sb.Append("\nmotion_status: ").Append(motion_status);
            sb.Append("\ntrajectory_num: ").Append(trajectory_num);
            sb.Append("\nerr_code: ").Append(err_code);
            return sb.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
