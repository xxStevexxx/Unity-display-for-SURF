Shader "SURF/RealSensePointCloud"
{
    Properties { _PointDiameter ("Point diameter (metres)", Float) = 0.008 }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        Pass
        {
            Cull Off ZWrite On ZTest LEqual
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            float _PointDiameter;
            struct Input { float4 vertex : POSITION; float4 color : COLOR; float2 uv : TEXCOORD0; };
            struct Output { float4 position : SV_POSITION; float4 color : COLOR; float2 uv : TEXCOORD0; };
            Output vert(Input input)
            {
                Output output;
                float3 view = UnityObjectToViewPos(input.vertex.xyz);
                view.xy += input.uv * (_PointDiameter * 0.5);
                output.position = mul(UNITY_MATRIX_P, float4(view, 1));
                output.color = input.color;
                #ifndef UNITY_COLORSPACE_GAMMA
                output.color.rgb = GammaToLinearSpace(output.color.rgb);
                #endif
                output.uv = input.uv;
                return output;
            }
            float4 frag(Output input) : SV_Target
            {
                clip(1 - dot(input.uv, input.uv));
                return float4(input.color.rgb, 1);
            }
            ENDCG
        }
    }
}
