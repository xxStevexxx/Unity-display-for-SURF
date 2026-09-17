using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using RosMessageTypes.Sensor;
using UnityEngine;

public static class RealSensePointCloudDecoder
{
    public enum Coordinates { RosFlu, Optical }

    [StructLayout(LayoutKind.Explicit)]
    private struct FloatBits
    {
        [FieldOffset(0)] public uint Bits;
        [FieldOffset(0)] public float Value;
    }

    public static bool Decode(PointCloud2Msg message, Coordinates coordinates, int maxPoints,
        List<Vector3> positions, List<Color32> colors, out string error)
    {
        positions.Clear();
        colors.Clear();
        error = null;
        if (message == null || message.data == null || message.fields == null || maxPoints < 1)
            return Fail("Missing cloud data or invalid point limit.", out error);
        if (message.width == 0 || message.height == 0)
            return true;
        ulong rowBytes = (ulong)message.width * message.point_step;
        ulong required = (ulong)(message.height - 1) * message.row_step + rowBytes;
        if (message.point_step == 0 || rowBytes > message.row_step || required > (ulong)message.data.LongLength)
            return Fail("Invalid point_step, row_step or truncated cloud data.", out error);

        PointFieldMsg x = Find(message, "x"), y = Find(message, "y"), z = Find(message, "z");
        if (!Valid(x, message.point_step, 7) || !Valid(y, message.point_step, 7) || !Valid(z, message.point_step, 7))
            return Fail("XYZ fields must be FLOAT32 values within point_step.", out error);
        PointFieldMsg rgb = Find(message, "rgb") ?? Find(message, "rgba");
        if (rgb != null && !Valid(rgb, message.point_step, 7) && !Valid(rgb, message.point_step, 6))
            return Fail("Packed RGB must be FLOAT32 or UINT32 within point_step.", out error);

        long count = (long)message.width * message.height;
        long stride = Math.Max(1, (count + maxPoints - 1) / maxPoints);
        for (long index = 0; index < count; index += stride)
        {
            int start = checked((int)((index / message.width) * message.row_step +
                (index % message.width) * message.point_step));
            float px = Float(message, start, x.offset);
            float py = Float(message, start, y.offset);
            float pz = Float(message, start, z.offset);
            if (!Finite(px) || !Finite(py) || !Finite(pz)) continue;
            // ROS FLU -> Unity RUF; optical RDF -> Unity RUF. Units remain metres.
            positions.Add(coordinates == Coordinates.RosFlu ? new Vector3(-py, pz, px) : new Vector3(px, -py, pz));
            uint color = rgb == null ? 0xccccccu : ReadUInt(message.data, start + (int)rgb.offset, message.is_bigendian);
            colors.Add(new Color32((byte)(color >> 16), (byte)(color >> 8), (byte)color, 255));
        }
        return true;
    }

    private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    private static bool Fail(string message, out string error) { error = message; return false; }
    private static PointFieldMsg Find(PointCloud2Msg message, string name)
    {
        foreach (var field in message.fields)
            if (field != null && field.name == name) return field;
        return null;
    }
    private static bool Valid(PointFieldMsg field, uint step, byte type) =>
        field != null && field.count >= 1 && field.datatype == type && (ulong)field.offset + 4 <= step;
    private static float Float(PointCloud2Msg message, int start, uint offset) =>
        new FloatBits { Bits = ReadUInt(message.data, start + (int)offset, message.is_bigendian) }.Value;
    private static uint ReadUInt(byte[] data, int offset, bool bigEndian)
    {
        if (bigEndian)
            return ((uint)data[offset] << 24) | ((uint)data[offset + 1] << 16) | ((uint)data[offset + 2] << 8) | data[offset + 3];
        return data[offset] | ((uint)data[offset + 1] << 8) | ((uint)data[offset + 2] << 16) | ((uint)data[offset + 3] << 24);
    }
}
