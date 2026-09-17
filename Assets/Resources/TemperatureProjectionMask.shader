Shader "Hidden/Piper/TemperatureProjectionMask"
{
    SubShader
    {
        Cull Back
        Blend Off
        ZTest LEqual
        CGINCLUDE
        #include "UnityCG.cginc"
        float4x4 _TemperatureViewProjection;
        float4 _TemperatureTint;
        struct Attributes { float4 vertex : POSITION; };
        struct Varyings { float4 position : SV_POSITION; };
        Varyings Vert(Attributes input)
        {
            Varyings output;
            output.position = mul(_TemperatureViewProjection, mul(unity_ObjectToWorld, input.vertex));
            return output;
        }
        float4 Frag(Varyings input) : SV_Target { return _TemperatureTint; }
        ENDCG
        Pass
        {
            Name "SceneDepth"
            ZWrite On
            ColorMask 0
            CGPROGRAM
            #pragma vertex Vert
            #pragma fragment Frag
            ENDCG
        }
        Pass
        {
            Name "VisibleTemperature"
            ZWrite Off
            CGPROGRAM
            #pragma vertex Vert
            #pragma fragment Frag
            ENDCG
        }
    }
    Fallback Off
}
