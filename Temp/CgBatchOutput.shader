Shader "Planet" {
	Properties {
		_MainTex ("Diffuse(RGB) Spec(A)", 2D) = "white" {}
		_BumpMap ("Bumpmap", 2D) = "bump" {}
		_RimColor ("Rim Color", Color) = (0.26,0.19,0.16,0.0)
		_RimPower ("Rim Power", Range(0.5,8.0)) = 3.0
		_SpecColor ("Specular Color", Color) = (0.5,0.5,0.5,1)
		_Shininess ("Shininess", Range (0.01, 1)) = 0.078125 
	}

	SubShader {	
		Tags { "RenderType" = "Opaque" }

			
	Pass {
		Name "FORWARD"
		Tags { "LightMode" = "ForwardBase" }
Program "vp" {
// Vertex combos: 9
//   opengl - ALU: 20 to 80
//   d3d9 - ALU: 21 to 83
//   d3d11 - ALU: 8 to 39, TEX: 0 to 0, FLOW: 1 to 1
//   d3d11_9x - ALU: 8 to 39, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_WorldSpaceLightPos0]
Vector 15 [unity_SHAr]
Vector 16 [unity_SHAg]
Vector 17 [unity_SHAb]
Vector 18 [unity_SHBr]
Vector 19 [unity_SHBg]
Vector 20 [unity_SHBb]
Vector 21 [unity_SHC]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 22 [unity_Scale]
Vector 23 [_MainTex_ST]
Vector 24 [_BumpMap_ST]
"!!ARBvp1.0
# 44 ALU
PARAM c[25] = { { 1 },
		state.matrix.mvp,
		program.local[5..24] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[22].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MUL R1, R0.xyzz, R0.yzzx;
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[17];
DP4 R2.y, R0, c[16];
DP4 R2.x, R0, c[15];
MUL R0.y, R2.w, R2.w;
DP4 R3.z, R1, c[20];
DP4 R3.y, R1, c[19];
DP4 R3.x, R1, c[18];
ADD R2.xyz, R2, R3;
MAD R0.x, R0, R0, -R0.y;
MUL R3.xyz, R0.x, c[21];
MOV R1.xyz, vertex.attrib[14];
MUL R0.xyz, vertex.normal.zxyw, R1.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R1.zxyw, -R0;
ADD result.texcoord[3].xyz, R2, R3;
MOV R0.xyz, c[13];
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[22].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[14];
DP4 R3.z, R1, c[11];
DP4 R3.x, R1, c[9];
DP4 R3.y, R1, c[10];
DP3 result.texcoord[1].y, R0, R2;
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[24].xyxy, c[24];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[23], c[23].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 44 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_WorldSpaceLightPos0]
Vector 14 [unity_SHAr]
Vector 15 [unity_SHAg]
Vector 16 [unity_SHAb]
Vector 17 [unity_SHBr]
Vector 18 [unity_SHBg]
Vector 19 [unity_SHBb]
Vector 20 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 21 [unity_Scale]
Vector 22 [_MainTex_ST]
Vector 23 [_BumpMap_ST]
"vs_2_0
; 47 ALU
def c24, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r1.xyz, v2, c21.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mul r1, r0.xyzz, r0.yzzx
mov r0.w, c24.x
dp4 r2.z, r0, c16
dp4 r2.y, r0, c15
dp4 r2.x, r0, c14
mul r0.y, r2.w, r2.w
dp4 r3.z, r1, c19
dp4 r3.y, r1, c18
dp4 r3.x, r1, c17
add r1.xyz, r2, r3
mad r0.x, r0, r0, -r0.y
mul r2.xyz, r0.x, c20
add oT3.xyz, r1, r2
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c13, r0
mov r0, c9
mov r1.w, c24.x
mov r1.xyz, c12
dp4 r4.y, c13, r0
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c21.w, -v0
mov r1, c8
dp4 r4.x, c13, r1
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
mad oT0.zw, v3.xyxy, c23.xyxy, c23
mad oT0.xy, v3, c22, c22.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 22 [_BumpMap_ST]
Vector 21 [_MainTex_ST]
Matrix 13 [_Object2World] 3
Matrix 16 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 9 [glstate_matrix_mvp] 4
Vector 4 [unity_SHAb]
Vector 3 [unity_SHAg]
Vector 2 [unity_SHAr]
Vector 7 [unity_SHBb]
Vector 6 [unity_SHBg]
Vector 5 [unity_SHBr]
Vector 8 [unity_SHC]
Vector 20 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 48.00 (36 instructions), vertex: 32, texture: 0,
//   sequencer: 20,  9 GPRs, 21 threads,
// Performance (if enough threads): ~48 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacomaaaaacciaaaaaaaaaaaaaaceaaaaaaaaaaaaaciaaaaaaaaa
aaaaaaaaaaaaacfiaaaaaabmaaaaacelpppoadaaaaaaaaapaaaaaabmaaaaaaaa
aaaaaceeaaaaabeiaaacaabgaaabaaaaaaaaabfeaaaaaaaaaaaaabgeaaacaabf
aaabaaaaaaaaabfeaaaaaaaaaaaaabhaaaacaaanaaadaaaaaaaaabiaaaaaaaaa
aaaaabjaaaacaabaaaaeaaaaaaaaabiaaaaaaaaaaaaaabjoaaacaaaaaaabaaaa
aaaaableaaaaaaaaaaaaabmeaaacaaabaaabaaaaaaaaabfeaaaaaaaaaaaaabnj
aaacaaajaaaeaaaaaaaaabiaaaaaaaaaaaaaabomaaacaaaeaaabaaaaaaaaabfe
aaaaaaaaaaaaabphaaacaaadaaabaaaaaaaaabfeaaaaaaaaaaaaacacaaacaaac
aaabaaaaaaaaabfeaaaaaaaaaaaaacanaaacaaahaaabaaaaaaaaabfeaaaaaaaa
aaaaacbiaaacaaagaaabaaaaaaaaabfeaaaaaaaaaaaaaccdaaacaaafaaabaaaa
aaaaabfeaaaaaaaaaaaaaccoaaacaaaiaaabaaaaaaaaabfeaaaaaaaaaaaaacdi
aaacaabeaaabaaaaaaaaabfeaaaaaaaafpechfgnhaengbhafpfdfeaaaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpepgcgkgfgdhedc
fhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgedcep
gcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaad
aaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphdda
aaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdeiebgcaahf
gogjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdeiecgcaa
hfgogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfdeiedaa
hfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaakl
aaaaaaaaaaaaacciaadbaaaiaaaaaaaaaaaaaaaaaaaadeieaaaaaaabaaaaaaae
aaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaahaadafaaiaaaapafaaaachbfb
aaafhcfcaaaihdfdaaaaaaccaaaabacdaaaaaabmaaaaaabnaaaababoaaaaaabp
aaaaaacaaaaabacbaaaabacmpaffeaafaaaabcaamcaaaaaaaaaaeaajaaaabcaa
meaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabjgabpbcaabcaaaaaaaaaagacf
caclbcaaccaaaaaaafpigaaaaaaaagiiaaaaaaaaafpifaaaaaaaagiiaaaaaaaa
afpibaaaaaaaaoiiaaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaacaabliiaa
kbagamaamiapaaacaamgiiaaklagalacmiapaaacaalbdejeklagakacmiapiado
aagmaadeklagajacmiahaaadaamamgmaalbcaabdmiahaaacaaleblaacbbdabaa
miahaaacaamamgleclbcabacmiahaaaeaalogfaaobabafaamiahaaaiaalelble
clbbaaadmialaaadaagfblaakbabbeaamiahaaahaalbleaakbadapaamiahaaai
aamagmleclbaaaaimiahaaaeabgflomaolabafaemiahaaacaalelbleclbbabac
miahaaacaamagmleclbaabacmiahaaaeaamablaaobaeafaamiahaaagabmablma
klaibeagmiahaaadaagmlemakladaoahmiahaaadaabllemakladanadmiabiaab
aaloloaapaagafaamiaciaabaaloloaapaaeagaamiaeiaabaaloloaapaagabaa
miabiaacaaloloaapaacafaamiaciaacaaloloaapaaeacaamiaeiaacaaloloaa
paacabaamiadiaaaaalalabkilaabfbfmiamiaaaaakmkmagilaabgbgceipadae
aalehcgmobadadiamiabaaacaadoanaagpacadaamiacaaacaadoanaagpadadaa
miaeaaacaadoanaagpaeadaamiabaaaaaakhkhaakpaeafaaaibcabaaaakhkhgm
kpaeagadaiceabaaaakhkhmgkpaeahadgeihaaaaaalologboaacaaabmiahiaad
aablmagfklaaaiaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Vector 465 [unity_SHAr]
Vector 464 [unity_SHAg]
Vector 463 [unity_SHAb]
Vector 462 [unity_SHBr]
Vector 461 [unity_SHBg]
Vector 460 [unity_SHBb]
Vector 459 [unity_SHC]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 458 [unity_Scale]
Vector 457 [_MainTex_ST]
Vector 456 [_BumpMap_ST]
"sce_vp_rsx // 42 instructions using 5 registers
[Configuration]
8
0000002a41050500
[Microcode]
672
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc00021c6c009ca20c013fc0c36041dffc
401f9c6c011c8800810040d560607f9c401f9c6c011c9808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00011c6c01d0a00d8286c0c360405ffc00011c6c01d0900d8286c0c360409ffc
00011c6c01d0800d8286c0c360411ffc00009c6c0150400c088600c360411ffc
00009c6c0150600c088600c360405ffc00001c6c0150500c088600c360403ffc
00021c6c0190a00c0086c0c360405ffc00021c6c0190900c0086c0c360409ffc
00021c6c0190800c0086c0c360411ffc00001c6c00800243011843436041dffc
00001c6c01000230812183630021dffc00019c6c011ca00c08bfc0e30041dffc
401f9c6c0140020c0106024360405fa4401f9c6c01400e0c0106024360411fa4
00009c6c0080007f80bfc04360403ffc00009c6c0040007f8086c08360409ffc
00021c6c00800e0c00bfc0836041dffc401f9c6c0140020c0106034360405fa0
401f9c6c01400e0c0686008360411fa000001c6c019cf00c0286c0c360405ffc
00001c6c019d000c0286c0c360409ffc00001c6c019d100c0286c0c360411ffc
00001c6c010000000280017fe0a03ffc00009c6c0080000d029a01436041fffc
401f9c6c0140000c0886024360409fa4401f9c6c0140000c0686044360409fa0
00011c6c01dcc00d8286c0c360405ffc00011c6c01dcd00d8286c0c360409ffc
00011c6c01dce00d8286c0c360411ffc00001c6c00c0000c0086c0830121dffc
00009c6c009cb07f808600c36041dffc401f9c6c00c0000c0286c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 128 used size, 8 vars
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 41 instructions, 4 temp regs, 0 temp arrays:
// ALU 23 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedcgagmcdphdadbcndibicnhehkmdjdjohabaaaaaanaahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcdeagaaaaeaaaabaa
inabaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacaeaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaahaaaaaa
kgiocaaaaaaaaaaaahaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadiaaaaaihcaabaaaaaaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaadiaaaaaihcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaaadaaaaaa
anaaaaaadcaaaaaklcaabaaaaaaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaa
aaaaaaaaegaibaaaabaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaa
aoaaaaaakgakbaaaaaaaaaaaegadbaaaaaaaaaaadgaaaaaficaabaaaaaaaaaaa
abeaaaaaaaaaiadpbbaaaaaibcaabaaaabaaaaaaegiocaaaacaaaaaabcaaaaaa
egaobaaaaaaaaaaabbaaaaaiccaabaaaabaaaaaaegiocaaaacaaaaaabdaaaaaa
egaobaaaaaaaaaaabbaaaaaiecaabaaaabaaaaaaegiocaaaacaaaaaabeaaaaaa
egaobaaaaaaaaaaadiaaaaahpcaabaaaacaaaaaajgacbaaaaaaaaaaaegakbaaa
aaaaaaaabbaaaaaibcaabaaaadaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaa
acaaaaaabbaaaaaiccaabaaaadaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaa
acaaaaaabbaaaaaiecaabaaaadaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaa
acaaaaaaaaaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaa
diaaaaahccaabaaaaaaaaaaabkaabaaaaaaaaaaabkaabaaaaaaaaaaadcaaaaak
bcaabaaaaaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaiaebaaaaaa
aaaaaaaadcaaaaakhccabaaaaeaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaa
aaaaaaaaegacbaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES


#ifdef VERTEX

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_8 = tmpvar_1.xyz;
  tmpvar_9 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_10;
  tmpvar_10[0].x = tmpvar_8.x;
  tmpvar_10[0].y = tmpvar_9.x;
  tmpvar_10[0].z = tmpvar_2.x;
  tmpvar_10[1].x = tmpvar_8.y;
  tmpvar_10[1].y = tmpvar_9.y;
  tmpvar_10[1].z = tmpvar_2.y;
  tmpvar_10[2].x = tmpvar_8.z;
  tmpvar_10[2].y = tmpvar_9.z;
  tmpvar_10[2].z = tmpvar_2.z;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_11;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_14;
  mediump vec4 normal_15;
  normal_15 = tmpvar_13;
  highp float vC_16;
  mediump vec3 x3_17;
  mediump vec3 x2_18;
  mediump vec3 x1_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAr, normal_15);
  x1_19.x = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAg, normal_15);
  x1_19.y = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAb, normal_15);
  x1_19.z = tmpvar_22;
  mediump vec4 tmpvar_23;
  tmpvar_23 = (normal_15.xyzz * normal_15.yzzx);
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBr, tmpvar_23);
  x2_18.x = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBg, tmpvar_23);
  x2_18.y = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBb, tmpvar_23);
  x2_18.z = tmpvar_26;
  mediump float tmpvar_27;
  tmpvar_27 = ((normal_15.x * normal_15.x) - (normal_15.y * normal_15.y));
  vC_16 = tmpvar_27;
  highp vec3 tmpvar_28;
  tmpvar_28 = (unity_SHC.xyz * vC_16);
  x3_17 = tmpvar_28;
  tmpvar_14 = ((x1_19 + x2_18) + x3_17);
  shlight_3 = tmpvar_14;
  tmpvar_6 = shlight_3;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_5;
  tmpvar_5 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), tmpvar_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  highp vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_11;
  lightDir_11 = xlv_TEXCOORD2;
  mediump vec3 viewDir_12;
  viewDir_12 = tmpvar_10;
  mediump vec4 c_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (tmpvar_5, lightDir_11));
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (tmpvar_5, normalize((lightDir_11 + viewDir_12))));
  nh_14 = tmpvar_16;
  mediump float tmpvar_17;
  tmpvar_17 = ((1.0 - dot (normalize(viewDir_12), tmpvar_5)) + dot (normalize(lightDir_11), tmpvar_5));
  highp vec3 tmpvar_18;
  tmpvar_18 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_15) + ((((_LightColor0.xyz * pow (nh_14, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * 2.0);
  c_13.xyz = tmpvar_18;
  highp vec3 tmpvar_19;
  tmpvar_19 = (c_13.xyz + (c_13.xyz * (pow (tmpvar_17, _RimPower) * _RimColor.xyz)));
  c_13.xyz = tmpvar_19;
  c_13.w = tmpvar_9;
  c_1 = c_13;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES


#ifdef VERTEX

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_8 = tmpvar_1.xyz;
  tmpvar_9 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_10;
  tmpvar_10[0].x = tmpvar_8.x;
  tmpvar_10[0].y = tmpvar_9.x;
  tmpvar_10[0].z = tmpvar_2.x;
  tmpvar_10[1].x = tmpvar_8.y;
  tmpvar_10[1].y = tmpvar_9.y;
  tmpvar_10[1].z = tmpvar_2.y;
  tmpvar_10[2].x = tmpvar_8.z;
  tmpvar_10[2].y = tmpvar_9.z;
  tmpvar_10[2].z = tmpvar_2.z;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_11;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_14;
  mediump vec4 normal_15;
  normal_15 = tmpvar_13;
  highp float vC_16;
  mediump vec3 x3_17;
  mediump vec3 x2_18;
  mediump vec3 x1_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAr, normal_15);
  x1_19.x = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAg, normal_15);
  x1_19.y = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAb, normal_15);
  x1_19.z = tmpvar_22;
  mediump vec4 tmpvar_23;
  tmpvar_23 = (normal_15.xyzz * normal_15.yzzx);
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBr, tmpvar_23);
  x2_18.x = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBg, tmpvar_23);
  x2_18.y = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBb, tmpvar_23);
  x2_18.z = tmpvar_26;
  mediump float tmpvar_27;
  tmpvar_27 = ((normal_15.x * normal_15.x) - (normal_15.y * normal_15.y));
  vC_16 = tmpvar_27;
  highp vec3 tmpvar_28;
  tmpvar_28 = (unity_SHC.xyz * vC_16);
  x3_17 = tmpvar_28;
  tmpvar_14 = ((x1_19 + x2_18) + x3_17);
  shlight_3 = tmpvar_14;
  tmpvar_6 = shlight_3;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 normal_5;
  normal_5.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_5.z = sqrt((1.0 - clamp (dot (normal_5.xy, normal_5.xy), 0.0, 1.0)));
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), normal_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  highp vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_11;
  lightDir_11 = xlv_TEXCOORD2;
  mediump vec3 viewDir_12;
  viewDir_12 = tmpvar_10;
  mediump vec4 c_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (normal_5, lightDir_11));
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (normal_5, normalize((lightDir_11 + viewDir_12))));
  nh_14 = tmpvar_16;
  mediump float tmpvar_17;
  tmpvar_17 = ((1.0 - dot (normalize(viewDir_12), normal_5)) + dot (normalize(lightDir_11), normal_5));
  highp vec3 tmpvar_18;
  tmpvar_18 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_15) + ((((_LightColor0.xyz * pow (nh_14, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * 2.0);
  c_13.xyz = tmpvar_18;
  highp vec3 tmpvar_19;
  tmpvar_19 = (c_13.xyz + (c_13.xyz * (pow (tmpvar_17, _RimPower) * _RimColor.xyz)));
  c_13.xyz = tmpvar_19;
  c_13.w = tmpvar_9;
  c_1 = c_13;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_WorldSpaceLightPos0]
Vector 14 [unity_SHAr]
Vector 15 [unity_SHAg]
Vector 16 [unity_SHAb]
Vector 17 [unity_SHBr]
Vector 18 [unity_SHBg]
Vector 19 [unity_SHBb]
Vector 20 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 21 [unity_Scale]
Vector 22 [_MainTex_ST]
Vector 23 [_BumpMap_ST]
"agal_vs
c24 1.0 0.0 0.0 0.0
[bc]
adaaaaaaabaaahacabaaaaoeaaaaaaaabfaaaappabaaaaaa mul r1.xyz, a1, c21.w
bcaaaaaaacaaaiacabaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r2.w, r1.xyzz, c5
bcaaaaaaaaaaabacabaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r0.x, r1.xyzz, c4
bcaaaaaaaaaaaeacabaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r0.z, r1.xyzz, c6
aaaaaaaaaaaaacacacaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r2.w
adaaaaaaabaaapacaaaaaakeacaaaaaaaaaaaacjacaaaaaa mul r1, r0.xyzz, r0.yzzx
aaaaaaaaaaaaaiacbiaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c24.x
bdaaaaaaacaaaeacaaaaaaoeacaaaaaabaaaaaoeabaaaaaa dp4 r2.z, r0, c16
bdaaaaaaacaaacacaaaaaaoeacaaaaaaapaaaaoeabaaaaaa dp4 r2.y, r0, c15
bdaaaaaaacaaabacaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 r2.x, r0, c14
adaaaaaaaaaaacacacaaaappacaaaaaaacaaaappacaaaaaa mul r0.y, r2.w, r2.w
bdaaaaaaadaaaeacabaaaaoeacaaaaaabdaaaaoeabaaaaaa dp4 r3.z, r1, c19
bdaaaaaaadaaacacabaaaaoeacaaaaaabcaaaaoeabaaaaaa dp4 r3.y, r1, c18
bdaaaaaaadaaabacabaaaaoeacaaaaaabbaaaaoeabaaaaaa dp4 r3.x, r1, c17
abaaaaaaabaaahacacaaaakeacaaaaaaadaaaakeacaaaaaa add r1.xyz, r2.xyzz, r3.xyzz
adaaaaaaadaaaiacaaaaaaaaacaaaaaaaaaaaaaaacaaaaaa mul r3.w, r0.x, r0.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaffacaaaaaa sub r0.x, r3.w, r0.y
adaaaaaaacaaahacaaaaaaaaacaaaaaabeaaaaoeabaaaaaa mul r2.xyz, r0.x, c20
abaaaaaaadaaahaeabaaaakeacaaaaaaacaaaakeacaaaaaa add v3.xyz, r1.xyzz, r2.xyzz
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r4.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c13, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
aaaaaaaaabaaaiacbiaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c24.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaeaaacacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c13, r0
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaaaaaahacacaaaakeacaaaaaabfaaaappabaaaaaa mul r0.xyz, r2.xyzz, c21.w
acaaaaaaacaaahacaaaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r0.xyzz, a0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacanaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c13, r1
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
adaaaaaaaaaaamacadaaaaeeaaaaaaaabhaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c23.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaabhaaaaoeabaaaaaa add v0.zw, r0.wwzw, c23
adaaaaaaaaaaadacadaaaaoeaaaaaaaabgaaaaoeabaaaaaa mul r0.xy, a3, c22
abaaaaaaaaaaadaeaaaaaafeacaaaaaabgaaaaooabaaaaaa add v0.xy, r0.xyyy, c22.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 128 used size, 8 vars
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 41 instructions, 4 temp regs, 0 temp arrays:
// ALU 23 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedddipifigalekfcfapecgaoikfambkmepabaaaaaakialaaaaaeaaaaaa
daaaaaaaaeaeaaaaeaakaaaaaialaaaaebgpgodjmmadaaaammadaaaaaaacpopp
faadaaaahmaaaaaaahaaceaaaaaahiaaaaaahiaaaaaaceaaabaahiaaaaaaagaa
acaaabaaaaaaaaaaabaaaeaaabaaadaaaaaaaaaaacaaaaaaabaaaeaaaaaaaaaa
acaabcaaahaaafaaaaaaaaaaadaaaaaaaeaaamaaaaaaaaaaadaaamaaadaabaaa
aaaaaaaaadaabaaaafaabdaaaaaaaaaaaaaaaaaaaaacpoppfbaaaaafbiaaapka
aaaaiadpaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaac
afaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapja
aeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaaeaaaaaeaaaaamoaadaaeeja
acaaeekaacaaoekaabaaaaacaaaaapiaaeaaoekaafaaaaadabaaahiaaaaaffia
beaaoekaaeaaaaaeabaaahiabdaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahia
bfaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiabgaaoekaaaaappiaaaaaoeia
aiaaaaadacaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaacaaoejaafaaaaad
acaaahiaabaanciaabaamjjaaeaaaaaeabaaahiaabaamjiaabaancjaacaaoeib
afaaaaadabaaahiaabaaoeiaabaappjaaiaaaaadacaaacoaabaaoeiaaaaaoeia
aiaaaaadacaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaadaaoekaafaaaaad
acaaahiaaaaaffiabeaaoekaaeaaaaaeaaaaaliabdaakekaaaaaaaiaacaakeia
aeaaaaaeaaaaahiabfaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeia
bgaaoekaaeaaaaaeaaaaahiaaaaaoeiabhaappkaaaaaoejbaiaaaaadabaaaboa
abaaoejaaaaaoeiaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoa
acaaoejaaaaaoeiaafaaaaadaaaaahiaacaaoejabhaappkaafaaaaadabaaahia
aaaaffiabbaaoekaaeaaaaaeaaaaaliabaaakekaaaaaaaiaabaakeiaaeaaaaae
aaaaahiabcaaoekaaaaakkiaaaaapeiaabaaaaacaaaaaiiabiaaaakaajaaaaad
abaaabiaafaaoekaaaaaoeiaajaaaaadabaaaciaagaaoekaaaaaoeiaajaaaaad
abaaaeiaahaaoekaaaaaoeiaafaaaaadacaaapiaaaaacjiaaaaakeiaajaaaaad
adaaabiaaiaaoekaacaaoeiaajaaaaadadaaaciaajaaoekaacaaoeiaajaaaaad
adaaaeiaakaaoekaacaaoeiaacaaaaadabaaahiaabaaoeiaadaaoeiaafaaaaad
aaaaaciaaaaaffiaaaaaffiaaeaaaaaeaaaaabiaaaaaaaiaaaaaaaiaaaaaffib
aeaaaaaeadaaahoaalaaoekaaaaaaaiaabaaoeiaafaaaaadaaaaapiaaaaaffja
anaaoekaaeaaaaaeaaaaapiaamaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapia
aoaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaapaaoekaaaaappjaaaaaoeia
aeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeia
ppppaaaafdeieefcdeagaaaaeaaaabaainabaaaafjaaaaaeegiocaaaaaaaaaaa
aiaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaa
bjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaa
adaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaa
gfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadhccabaaa
aeaaaaaagiaaaaacaeaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaa
agaaaaaaogikcaaaaaaaaaaaagaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaa
adaaaaaaagiecaaaaaaaaaaaahaaaaaakgiocaaaaaaaaaaaahaaaaaadiaaaaah
hcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaaj
hcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaa
aeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaa
abaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaa
aaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaah
eccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaa
adaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaihcaabaaaaaaaaaaa
egbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaaaaaaaaa
egiicaaaadaaaaaaamaaaaaaagaabaaaaaaaaaaaegaibaaaabaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaaaaaaaaaegadbaaa
aaaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadpbbaaaaaibcaabaaa
abaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaaaaaaaaabbaaaaaiccaabaaa
abaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaaaaaaaaabbaaaaaiecaabaaa
abaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaaaaaaaaadiaaaaahpcaabaaa
acaaaaaajgacbaaaaaaaaaaaegakbaaaaaaaaaaabbaaaaaibcaabaaaadaaaaaa
egiocaaaacaaaaaabfaaaaaaegaobaaaacaaaaaabbaaaaaiccaabaaaadaaaaaa
egiocaaaacaaaaaabgaaaaaaegaobaaaacaaaaaabbaaaaaiecaabaaaadaaaaaa
egiocaaaacaaaaaabhaaaaaaegaobaaaacaaaaaaaaaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaaegacbaaaadaaaaaadiaaaaahccaabaaaaaaaaaaabkaabaaa
aaaaaaaabkaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
akaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaadcaaaaakhccabaaaaeaaaaaa
egiccaaaacaaaaaabiaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaadoaaaaab
ejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
aaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaa
kjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaalaaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaaaaaaaaaaadaaaaaa
aeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaa
faepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaafeeffiedepepfcee
aaedepemepfcaaklepfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaa
abaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahaiaaaa
imaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahaiaaaaimaaaaaaadaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffied
epepfceeaaklklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 400
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 415
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 384
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 407
#line 424
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 444
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 136
mediump vec3 ShadeSH9( in mediump vec4 normal ) {
    mediump vec3 x1;
    mediump vec3 x2;
    mediump vec3 x3;
    x1.x = dot( unity_SHAr, normal);
    #line 140
    x1.y = dot( unity_SHAg, normal);
    x1.z = dot( unity_SHAb, normal);
    mediump vec4 vB = (normal.xyzz * normal.yzzx);
    x2.x = dot( unity_SHBr, vB);
    #line 144
    x2.y = dot( unity_SHBg, vB);
    x2.z = dot( unity_SHBb, vB);
    highp float vC = ((normal.x * normal.x) - (normal.y * normal.y));
    x3 = (unity_SHC.xyz * vC);
    #line 148
    return ((x1 + x2) + x3);
}
#line 426
v2f_surf vert_surf( in appdata_full v ) {
    #line 428
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 432
    highp vec3 worldN = (mat3( _Object2World) * (v.normal * unity_Scale.w));
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    #line 436
    o.lightDir = lightDir;
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    highp vec3 shlight = ShadeSH9( vec4( worldN, 1.0));
    #line 440
    o.vlight = shlight;
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out highp vec3 xlv_TEXCOORD1;
out lowp vec3 xlv_TEXCOORD2;
out lowp vec3 xlv_TEXCOORD3;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD2 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD3 = vec3(xl_retval.vlight);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 400
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 415
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 384
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 407
#line 424
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 444
#line 387
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    #line 389
    mediump vec3 h = normalize((lightDir + viewDir));
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    #line 393
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    #line 397
    c.w = s.Alpha;
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 407
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 411
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 444
lowp vec4 frag_surf( in v2f_surf IN ) {
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    #line 448
    surfIN.uv_BumpMap = IN.pack0.zw;
    surfIN.viewDir = IN.viewDir;
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    #line 452
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    #line 456
    surf( surfIN, o);
    lowp float atten = 1.0;
    lowp vec4 c = vec4( 0.0);
    c = LightingSimpleSpecular( o, IN.lightDir, normalize(IN.viewDir), atten);
    #line 460
    c.xyz += (o.Albedo * IN.vlight);
    c.xyz += o.Emission;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in highp vec3 xlv_TEXCOORD1;
in lowp vec3 xlv_TEXCOORD2;
in lowp vec3 xlv_TEXCOORD3;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD1);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD2);
    xlt_IN.vlight = vec3(xlv_TEXCOORD3);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 13 [_WorldSpaceCameraPos]
Matrix 9 [_World2Object]
Vector 15 [unity_Scale]
Vector 16 [unity_LightmapST]
Vector 17 [_MainTex_ST]
Vector 18 [_BumpMap_ST]
"!!ARBvp1.0
# 20 ALU
PARAM c[19] = { { 1 },
		state.matrix.mvp,
		program.local[5..18] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R1.xyz, R0, vertex.attrib[14].w;
MOV R0.xyz, c[13];
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[15].w, -vertex.position;
DP3 result.texcoord[1].y, R0, R1;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[18].xyxy, c[18];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[17], c[17].zwzw;
MAD result.texcoord[2].xy, vertex.texcoord[1], c[16], c[16].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 20 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Matrix 8 [_World2Object]
Vector 13 [unity_Scale]
Vector 14 [unity_LightmapST]
Vector 15 [_MainTex_ST]
Vector 16 [_BumpMap_ST]
"vs_2_0
; 21 ALU
def c17, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r1.xyz, r0, v1.w
mov r0.xyz, c12
mov r0.w, c17.x
dp4 r2.z, r0, c10
dp4 r2.x, r0, c8
dp4 r2.y, r0, c9
mad r0.xyz, r2, c13.w, -v0
dp3 oT1.y, r0, r1
dp3 oT1.z, v2, r0
dp3 oT1.x, r0, v1
mad oT0.zw, v3.xyxy, c16.xyxy, c16
mad oT0.xy, v3, c15, c15.zwzw
mad oT2.xy, v4, c14, c14.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 12 [_BumpMap_ST]
Vector 11 [_MainTex_ST]
Matrix 5 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 1 [glstate_matrix_mvp] 4
Vector 10 [unity_LightmapST]
Vector 9 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 22.67 (17 instructions), vertex: 64, texture: 0,
//   sequencer: 14,  6 GPRs, 30 threads,
// Performance (if enough threads): ~64 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaaboeaaaaabeeaaaaaaaaaaaaaaceaaaaaaaaaaaaabieaaaaaaaa
aaaaaaaaaaaaabfmaaaaaabmaaaaabeppppoadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabeiaaaaaakiaaacaaamaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaal
aaabaaaaaaaaaaleaaaaaaaaaaaaaanaaaacaaafaaaeaaaaaaaaaaoaaaaaaaaa
aaaaaapaaaacaaaaaaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaabaaaeaaaa
aaaaaaoaaaaaaaaaaaaaabclaaacaaakaaabaaaaaaaaaaleaaaaaaaaaaaaabdm
aaacaaajaaabaaaaaaaaaaleaaaaaaaafpechfgnhaengbhafpfdfeaaaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpfhgphcgmgedcep
gcgkgfgdheaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgefdha
gbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaaaaaaaaaa
ghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpemgjghgihegngb
hafdfeaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfdd
codaaaklaaaaaaaaaaaaabeeaacbaaafaaaaaaaaaaaaaaaaaaaacegdaaaaaaab
aaaaaaafaaaaaaagaaaaacjaaabaaaaeaaaagaafaaaadaagaaaafaahaadbfaai
aaaapafaaaachbfbaaafdcfcaaaaaabiaaaababjaaaaaabeaaaaaabfaaaababg
aaaababhpbfffaaeaaaabcabmcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaan
gabdbcaabcaaaaaaaaaababjaaaaccaaaaaaaaaaafpicaaaaaaaagiiaaaaaaaa
afpieaaaaaaaagiiaaaaaaaaafpibaaaaaaaaoiiaaaaaaaaafpiaaaaaaaaaoeh
aaaaaaaaafpiaaaaaaaaadpiaaaaaaaamiapaaadaabliiaakbacaeaamiapaaad
aamgiiaaklacadadmiapaaadaalbdejeklacacadmiapiadoaagmaadeklacabad
miahaaafaamamgmaalahaaaimiahaaadaalogfaaobabaeaamiahaaafaalelble
clagaaafmiahaaafaamagmleclafaaafmiahaaadabgflomaolabaeadmiahaaad
aamablaaobadaeaamiahaaacabmablmaklafajacmiabiaabaaloloaapaacaeaa
miaciaabaaloloaapaadacaamiaeiaabaaloloaapaacabaamiadiaacaabilabk
ilaaakakmiadiaaaaamflabkilaaalalmiamiaaaaapbkmagilaaamamaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_WorldSpaceCameraPos]
Matrix 264 [_World2Object]
Vector 466 [unity_Scale]
Vector 465 [unity_LightmapST]
Vector 464 [_MainTex_ST]
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 19 instructions using 3 registers
[Configuration]
8
0000001343050300
[Microcode]
304
00009c6c00400e0c0106c0836041dffc00011c6c005d300c0186c0836041dffc
401f9c6c011cf800810040d560607f9c401f9c6c011d0808010400d740619f9c
401f9c6c011d1908010400d740619fa4401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c0190a00c0486c0c360405ffc
00001c6c0190900c0486c0c360409ffc00001c6c0190800c0486c0c360411ffc
00011c6c00800243011841436041dffc00009c6c01000230812181630121dffc
00001c6c011d200c00bfc0e30041dffc00009c6c00800e0c02bfc0836041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
401f9c6c0140000c0086014360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 144 // 144 used size, 9 vars
Vector 96 [unity_LightmapST] 4
Vector 112 [_MainTex_ST] 4
Vector 128 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 19 instructions, 2 temp regs, 0 temp arrays:
// ALU 8 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedgihjligonkmienkeammlfhjndmgppdlhabaaaaaanaaeaaaaadaaaaaa
cmaaaaaapeaaaaaahmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaa
aiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaheaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
adamaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
emadaaaaeaaaabaandaaaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaa
gfaaaaaddccabaaaadaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaacaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaahaaaaaaogikcaaaaaaaaaaaahaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaaiaaaaaakgiocaaaaaaaaaaa
aiaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaa
abaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
acaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaacaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaacaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
aaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaacaaaaaabdaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaacaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
dcaaaaaldccabaaaadaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaaagaaaaaa
ogikcaaaaaaaaaaaagaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES


#ifdef VERTEX

varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  tmpvar_4 = tmpvar_1.xyz;
  tmpvar_5 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_6;
  tmpvar_6[0].x = tmpvar_4.x;
  tmpvar_6[0].y = tmpvar_5.x;
  tmpvar_6[0].z = tmpvar_2.x;
  tmpvar_6[1].x = tmpvar_4.y;
  tmpvar_6[1].y = tmpvar_5.y;
  tmpvar_6[1].z = tmpvar_2.y;
  tmpvar_6[2].x = tmpvar_4.z;
  tmpvar_6[2].y = tmpvar_5.z;
  tmpvar_6[2].z = tmpvar_2.z;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = _WorldSpaceCameraPos;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_7).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec3 tmpvar_4;
  tmpvar_4 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  highp float tmpvar_5;
  tmpvar_5 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), tmpvar_4), 0.0, 1.0));
  rim_3 = tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_6 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_6;
  c_1.xyz = (texture2D (_MainTex, xlv_TEXCOORD0.xy).xyz * (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD2).xyz));
  c_1.w = texture2D (_MainTex, xlv_TEXCOORD0.xy).w;
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES


#ifdef VERTEX

varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  tmpvar_4 = tmpvar_1.xyz;
  tmpvar_5 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_6;
  tmpvar_6[0].x = tmpvar_4.x;
  tmpvar_6[0].y = tmpvar_5.x;
  tmpvar_6[0].z = tmpvar_2.x;
  tmpvar_6[1].x = tmpvar_4.y;
  tmpvar_6[1].y = tmpvar_5.y;
  tmpvar_6[1].z = tmpvar_2.y;
  tmpvar_6[2].x = tmpvar_4.z;
  tmpvar_6[2].y = tmpvar_5.z;
  tmpvar_6[2].z = tmpvar_2.z;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = _WorldSpaceCameraPos;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_7).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec3 normal_4;
  normal_4.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_4.z = sqrt((1.0 - clamp (dot (normal_4.xy, normal_4.xy), 0.0, 1.0)));
  highp float tmpvar_5;
  tmpvar_5 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), normal_4), 0.0, 1.0));
  rim_3 = tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_6 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (unity_Lightmap, xlv_TEXCOORD2);
  c_1.xyz = (texture2D (_MainTex, xlv_TEXCOORD0.xy).xyz * ((8.0 * tmpvar_7.w) * tmpvar_7.xyz));
  c_1.w = texture2D (_MainTex, xlv_TEXCOORD0.xy).w;
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Matrix 8 [_World2Object]
Vector 13 [unity_Scale]
Vector 14 [unity_LightmapST]
Vector 15 [_MainTex_ST]
Vector 16 [_BumpMap_ST]
"agal_vs
c17 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaacaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r2.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r2.xyzz, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r1.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c12
aaaaaaaaaaaaaiacbbaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c17.x
bdaaaaaaacaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r0, c10
bdaaaaaaacaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r0, c8
bdaaaaaaacaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r0, c9
adaaaaaaacaaahacacaaaakeacaaaaaaanaaaappabaaaaaa mul r2.xyz, r2.xyzz, c13.w
acaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r2.xyzz, a0
bcaaaaaaabaaacaeaaaaaakeacaaaaaaabaaaakeacaaaaaa dp3 v1.y, r0.xyzz, r1.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v1.z, a1, r0.xyzz
bcaaaaaaabaaabaeaaaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r0.xyzz, a5
adaaaaaaaaaaamacadaaaaeeaaaaaaaabaaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c16.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaabaaaaaoeabaaaaaa add v0.zw, r0.wwzw, c16
adaaaaaaaaaaadacadaaaaoeaaaaaaaaapaaaaoeabaaaaaa mul r0.xy, a3, c15
abaaaaaaaaaaadaeaaaaaafeacaaaaaaapaaaaooabaaaaaa add v0.xy, r0.xyyy, c15.zwzw
adaaaaaaaaaaadacaeaaaaoeaaaaaaaaaoaaaaoeabaaaaaa mul r0.xy, a4, c14
abaaaaaaacaaadaeaaaaaafeacaaaaaaaoaaaaooabaaaaaa add v2.xy, r0.xyyy, c14.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.zw, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 144 // 144 used size, 9 vars
Vector 96 [unity_LightmapST] 4
Vector 112 [_MainTex_ST] 4
Vector 128 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 19 instructions, 2 temp regs, 0 temp arrays:
// ALU 8 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedconhgcnkmbhelebgmdipcjlffoaeibdmabaaaaaapiagaaaaaeaaaaaa
daaaaaaafeacaaaakiafaaaahaagaaaaebgpgodjbmacaaaabmacaaaaaaacpopp
meabaaaafiaaaaaaaeaaceaaaaaafeaaaaaafeaaaaaaceaaabaafeaaaaaaagaa
adaaabaaaaaaaaaaabaaaeaaabaaaeaaaaaaaaaaacaaaaaaaeaaafaaaaaaaaaa
acaabaaaafaaajaaaaaaaaaaaaaaaaaaaaacpoppbpaaaaacafaaaaiaaaaaapja
bpaaaaacafaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadia
adaaapjabpaaaaacafaaaeiaaeaaapjaaeaaaaaeaaaaadoaadaaoejaacaaoeka
acaaookaaeaaaaaeaaaaamoaadaaeejaadaaeekaadaaoekaaeaaaaaeacaaadoa
aeaaoejaabaaoekaabaaookaabaaaaacaaaaahiaaeaaoekaafaaaaadabaaahia
aaaaffiaakaaoekaaeaaaaaeaaaaaliaajaakekaaaaaaaiaabaakeiaaeaaaaae
aaaaahiaalaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeiaamaaoeka
aeaaaaaeaaaaahiaaaaaoeiaanaappkaaaaaoejbaiaaaaadabaaaboaabaaoeja
aaaaoeiaabaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancja
aeaaaaaeabaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeia
abaappjaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoeja
aaaaoeiaafaaaaadaaaaapiaaaaaffjaagaaoekaaeaaaaaeaaaaapiaafaaoeka
aaaaaajaaaaaoeiaaeaaaaaeaaaaapiaahaaoekaaaaakkjaaaaaoeiaaeaaaaae
aaaaapiaaiaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoeka
aaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaafdeieefcemadaaaaeaaaabaa
ndaaaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaa
adaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaa
gfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaaddccabaaa
adaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaacaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaacaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pccabaaaaaaaaaaaegiocaaaacaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaa
ahaaaaaaogikcaaaaaaaaaaaahaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaa
adaaaaaaagiecaaaaaaaaaaaaiaaaaaakgiocaaaaaaaaaaaaiaaaaaadiaaaaah
hcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaaj
hcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaacaaaaaabbaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaacaaaaaabaaaaaaaagiacaaaabaaaaaa
aeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaacaaaaaa
bcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaacaaaaaabdaaaaaadcaaaaalhcaabaaa
abaaaaaaegacbaaaabaaaaaapgipcaaaacaaaaaabeaaaaaaegbcbaiaebaaaaaa
aaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaah
eccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadcaaaaaldccabaaa
adaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaa
agaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
laaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
afaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaa
feeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaaaiaaaaaa
giaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaiaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaadamaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 400
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 415
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    highp vec2 lmap;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 384
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 407
#line 423
uniform highp vec4 unity_LightmapST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
uniform sampler2D unity_Lightmap;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 426
v2f_surf vert_surf( in appdata_full v ) {
    #line 428
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 432
    o.lmap.xy = ((v.texcoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
    highp vec3 worldN = (mat3( _Object2World) * (v.normal * unity_Scale.w));
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    #line 436
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    #line 440
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out highp vec3 xlv_TEXCOORD1;
out highp vec2 xlv_TEXCOORD2;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD2 = vec2(xl_retval.lmap);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 400
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 415
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    highp vec2 lmap;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 384
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 407
#line 423
uniform highp vec4 unity_LightmapST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
uniform sampler2D unity_Lightmap;
#line 176
lowp vec3 DecodeLightmap( in lowp vec4 color ) {
    #line 178
    return (2.0 * color.xyz);
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 407
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 411
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 443
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 445
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    surfIN.viewDir = IN.viewDir;
    #line 449
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    #line 453
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp float atten = 1.0;
    #line 457
    lowp vec4 c = vec4( 0.0);
    lowp vec4 lmtex = texture( unity_Lightmap, IN.lmap.xy);
    lowp vec3 lm = DecodeLightmap( lmtex);
    c.xyz += (o.Albedo * lm);
    #line 461
    c.w = o.Alpha;
    c.xyz += o.Emission;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in highp vec3 xlv_TEXCOORD1;
in highp vec2 xlv_TEXCOORD2;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD1);
    xlt_IN.lmap = vec2(xlv_TEXCOORD2);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_ProjectionParams]
Vector 15 [_WorldSpaceLightPos0]
Vector 16 [unity_SHAr]
Vector 17 [unity_SHAg]
Vector 18 [unity_SHAb]
Vector 19 [unity_SHBr]
Vector 20 [unity_SHBg]
Vector 21 [unity_SHBb]
Vector 22 [unity_SHC]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 23 [unity_Scale]
Vector 24 [_MainTex_ST]
Vector 25 [_BumpMap_ST]
"!!ARBvp1.0
# 49 ALU
PARAM c[26] = { { 1, 0.5 },
		state.matrix.mvp,
		program.local[5..25] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[23].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MUL R1, R0.xyzz, R0.yzzx;
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[18];
DP4 R2.y, R0, c[17];
DP4 R2.x, R0, c[16];
MUL R0.y, R2.w, R2.w;
DP4 R3.z, R1, c[21];
DP4 R3.y, R1, c[20];
DP4 R3.x, R1, c[19];
ADD R2.xyz, R2, R3;
MAD R0.x, R0, R0, -R0.y;
MUL R3.xyz, R0.x, c[22];
MOV R1.xyz, vertex.attrib[14];
MUL R0.xyz, vertex.normal.zxyw, R1.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R1.zxyw, -R0;
ADD result.texcoord[3].xyz, R2, R3;
MOV R0.w, c[0].x;
MOV R0.xyz, c[13];
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[23].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[15];
DP4 R3.z, R1, c[11];
DP4 R3.x, R1, c[9];
DP4 R3.y, R1, c[10];
DP3 result.texcoord[1].y, R0, R2;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.w, vertex.position, c[4];
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
ADD result.texcoord[4].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[4].zw, R0;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[25].xyxy, c[25];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[24], c[24].zwzw;
END
# 49 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_ScreenParams]
Vector 15 [_WorldSpaceLightPos0]
Vector 16 [unity_SHAr]
Vector 17 [unity_SHAg]
Vector 18 [unity_SHAb]
Vector 19 [unity_SHBr]
Vector 20 [unity_SHBg]
Vector 21 [unity_SHBb]
Vector 22 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 23 [unity_Scale]
Vector 24 [_MainTex_ST]
Vector 25 [_BumpMap_ST]
"vs_2_0
; 52 ALU
def c26, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r1.xyz, v2, c23.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mul r1, r0.xyzz, r0.yzzx
mov r0.w, c26.x
dp4 r2.z, r0, c18
dp4 r2.y, r0, c17
dp4 r2.x, r0, c16
mul r0.y, r2.w, r2.w
dp4 r3.z, r1, c21
dp4 r3.y, r1, c20
dp4 r3.x, r1, c19
add r1.xyz, r2, r3
mad r0.x, r0, r0, -r0.y
mul r2.xyz, r0.x, c22
add oT3.xyz, r1, r2
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c15, r0
mov r0, c9
dp4 r4.y, c15, r0
mov r1.w, c26.x
mov r1.xyz, c12
dp4 r0.w, v0, c3
dp4 r0.z, v0, c2
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c23.w, -v0
mov r1, c8
dp4 r4.x, c15, r1
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c26.y
mul r1.y, r1, c13.x
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
mad oT4.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT4.zw, r0
mad oT0.zw, v3.xyxy, c25.xyxy, c25
mad oT0.xy, v3, c24, c24.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 24 [_BumpMap_ST]
Vector 23 [_MainTex_ST]
Matrix 15 [_Object2World] 3
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 18 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 3 [_WorldSpaceLightPos0]
Matrix 11 [glstate_matrix_mvp] 4
Vector 6 [unity_SHAb]
Vector 5 [unity_SHAg]
Vector 4 [unity_SHAr]
Vector 9 [unity_SHBb]
Vector 8 [unity_SHBg]
Vector 7 [unity_SHBr]
Vector 10 [unity_SHC]
Vector 22 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 53.33 (40 instructions), vertex: 32, texture: 0,
//   sequencer: 20,  10 GPRs, 18 threads,
// Performance (if enough threads): ~53 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadgiaaaaacjiaaaaaaaaaaaaaaceaaaaacmiaaaaacpaaaaaaaaa
aaaaaaaaaaaaackaaaaaaabmaaaaacjdpppoadaaaaaaaabbaaaaaabmaaaaaaaa
aaaaacimaaaaabhaaaacaabiaaabaaaaaaaaabhmaaaaaaaaaaaaabimaaacaabh
aaabaaaaaaaaabhmaaaaaaaaaaaaabjiaaacaaapaaadaaaaaaaaabkiaaaaaaaa
aaaaabliaaacaaabaaabaaaaaaaaabhmaaaaaaaaaaaaabmkaaacaaacaaabaaaa
aaaaabhmaaaaaaaaaaaaabniaaacaabcaaaeaaaaaaaaabkiaaaaaaaaaaaaabog
aaacaaaaaaabaaaaaaaaabpmaaaaaaaaaaaaacamaaacaaadaaabaaaaaaaaabhm
aaaaaaaaaaaaaccbaaacaaalaaaeaaaaaaaaabkiaaaaaaaaaaaaacdeaaacaaag
aaabaaaaaaaaabhmaaaaaaaaaaaaacdpaaacaaafaaabaaaaaaaaabhmaaaaaaaa
aaaaacekaaacaaaeaaabaaaaaaaaabhmaaaaaaaaaaaaacffaaacaaajaaabaaaa
aaaaabhmaaaaaaaaaaaaacgaaaacaaaiaaabaaaaaaaaabhmaaaaaaaaaaaaacgl
aaacaaahaaabaaaaaaaaabhmaaaaaaaaaaaaachgaaacaaakaaabaaaaaaaaabhm
aaaaaaaaaaaaaciaaaacaabgaaabaaaaaaaaabhmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaa
fpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaa
fpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfa
gphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdhagbgdgfem
gjghgihefagphddaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhe
hjfpfdeiebgcaahfgogjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogj
hehjfpfdeiecgcaahfgogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgo
gjhehjfpfdeiedaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodc
dadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeaapmaaba
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaacfiaaebaaajaaaaaaaa
aaaaaaaaaaaaeekfaaaaaaabaaaaaaaeaaaaaaalaaaaacjaaabaaaafaaaagaag
aaaadaahaadafaaiaaaapafaaaachbfbaaafhcfcaaaihdfdaaajpefeaaaaaacf
aaaabacgaaaaaabpaaaaaacaaaaabacbaaaaaaccaaaaaacdaaaabaceaaaabada
aaaaaaboaaaabacoaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaapaffeaafaaaabcaamcaaaaaaaaaafaajaaaabcaameaaaaaa
aaaagaaogabebcaabcaaaaaaaaaagabkgacabcaabcaaaaaaaaaagacgfacmbcaa
ccaaaaaaafpihaaaaaaaagiiaaaaaaaaafpigaaaaaaaagiiaaaaaaaaafpibaaa
aaaaaeehaaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaacaabliiaakbahaoaa
miapaaacaamgnapiklahanacmiapaaacaalbdepiklahamacmiapaaaiaagmnaje
klahalacmiapiadoaananaaaocaiaiaamiahaaadaamamgmaalbeaabfmiahaaac
aaleblaacbbfadaamiahaaacaamamgleclbeadacmiahaaafaamdgfaaobabagaa
miahaaajaalelbleclbdaaadmialaaadaalkblaakbabbgaamiahaaaeaalbleaa
kbadbbaamiahaaajaamagmleclbcaaajmiahaaafablklomaolabagafmiahaaac
aalelbleclbdadacmiahaaacaamagmleclbcadacceihaeafaamablgmobafagia
miahaaahabmablmaklajbgahmiahaaadaagmlemakladbaaemiahaaaeaabllema
kladapadaibhabadaamagmggkbaippaemiamiaaeaanlnlaaocaiaiaamiabiaab
aaloloaapaahagaamiaciaabaaloloaapaafahaamiaeiaabaalomdaapaahabaa
miabiaacaaloloaapaacagaamiaciaacaaloloaapaafacaamiaeiaacaalomdaa
paacabaamiadiaaaaalalabkilaabhbhmiamiaaaaakmkmagilaabibiaicbabac
aadoanmbgpaeaeaeaiecabacaadoanlbgpafaeaeaiieabacaadoanlmgpagaeae
miabaaaaaakhkhaakpabahaamiacaaaaaakhkhaakpabaiaaaibeabaaaakhkhgm
kpabajaeaiciabadaalbgmmgkbadabaemiadiaaeaamgbkbikladacadgeihaaaa
aalologboaacaaabmiahiaadaablmagfklaaakaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
Vector 465 [_WorldSpaceLightPos0]
Vector 464 [unity_SHAr]
Vector 463 [unity_SHAg]
Vector 462 [unity_SHAb]
Vector 461 [unity_SHBr]
Vector 460 [unity_SHBg]
Vector 459 [unity_SHBb]
Vector 458 [unity_SHC]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 457 [unity_Scale]
Vector 456 [_MainTex_ST]
Vector 455 [_BumpMap_ST]
"sce_vp_rsx // 47 instructions using 7 registers
[Configuration]
8
0000002f41050700
[Defaults]
1
454 1
3f000000
[Microcode]
752
00009c6c005d100d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc00021c6c009c920c013fc0c36041dffc
401f9c6c011c7800810040d560607f9c401f9c6c011c8808010400d740619f9c
00001c6c01d0300d8106c0c360403ffc00001c6c01d0200d8106c0c360405ffc
00001c6c01d0100d8106c0c360409ffc00001c6c01d0000d8106c0c360411ffc
00029c6c01d0a00d8286c0c360405ffc00029c6c01d0900d8286c0c360409ffc
00029c6c01d0800d8286c0c360411ffc00031c6c0150400c088600c360411ffc
00031c6c0150600c088600c360405ffc00009c6c0150500c088600c360403ffc
00009c6c0190a00c0486c0c360405ffc00009c6c0190900c0486c0c360409ffc
00009c6c0190800c0486c0c360411ffc00011c6c00800243011843436041dffc
00011c6c01000230812183630121dffc401f9c6c0040000d8086c0836041ff80
401f9c6c004000558086c08360407fac00009c6c011c900c02bfc0e30041dffc
00001c6c009c600e008000c36041dffc401f9c6c0140020c0106054360405fa4
401f9c6c01400e0c0106054360411fa400001c6c009d202a808000c360409ffc
00001c6c0080007f82bfc14360403ffc00031c6c0040007f8286c08360409ffc
401f9c6c00c000080086c09540219fac00011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106014360405fa0401f9c6c01400e0c0286008360411fa0
00001c6c019ce00c0c86c0c360405ffc00001c6c019cf00c0c86c0c360409ffc
00001c6c019d000c0c86c0c360411ffc00001c6c010000000c80067fe0203ffc
00019c6c0080000d0c9a06436041fffc401f9c6c0140000c0486054360409fa4
401f9c6c0140000c0286024360409fa000009c6c01dcb00d8686c0c360405ffc
00009c6c01dcc00d8686c0c360409ffc00009c6c01dcd00d8686c0c360411ffc
00001c6c00c0000c0086c08300a1dffc00009c6c009ca07f808600c36041dffc
401f9c6c00c0000c0286c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 46 instructions, 5 temp regs, 0 temp arrays:
// ALU 26 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedeggbeheggghjepcdjoicghgpaahmcdepabaaaaaaiaaiaaaaadaaaaaa
cmaaaaaapeaaaaaakmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefcmmagaaaaeaaaabaaldabaaaafjaaaaae
egiocaaaaaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaae
egiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
pccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaadhccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaacafaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaalaaaaaakgiocaaaaaaaaaaa
alaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaa
abaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaa
aaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaabdaaaaaa
dcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaa
diaaaaajhcaabaaaacaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaal
hcaabaaaacaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaacaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadiaaaaai
hcaabaaaabaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaai
hcaabaaaacaaaaaafgafbaaaabaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaak
lcaabaaaabaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaabaaaaaaegaibaaa
acaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaa
abaaaaaaegadbaaaabaaaaaadgaaaaaficaabaaaabaaaaaaabeaaaaaaaaaiadp
bbaaaaaibcaabaaaacaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaabaaaaaa
bbaaaaaiccaabaaaacaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaabaaaaaa
bbaaaaaiecaabaaaacaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaabaaaaaa
diaaaaahpcaabaaaadaaaaaajgacbaaaabaaaaaaegakbaaaabaaaaaabbaaaaai
bcaabaaaaeaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaadaaaaaabbaaaaai
ccaabaaaaeaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaadaaaaaabbaaaaai
ecaabaaaaeaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaadaaaaaaaaaaaaah
hcaabaaaacaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaahccaabaaa
abaaaaaabkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaakbcaabaaaabaaaaaa
akaabaaaabaaaaaaakaabaaaabaaaaaabkaabaiaebaaaaaaabaaaaaadcaaaaak
hccabaaaaeaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaaabaaaaaaegacbaaa
acaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaafaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaafaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES


#ifdef VERTEX

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_8 = tmpvar_1.xyz;
  tmpvar_9 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_10;
  tmpvar_10[0].x = tmpvar_8.x;
  tmpvar_10[0].y = tmpvar_9.x;
  tmpvar_10[0].z = tmpvar_2.x;
  tmpvar_10[1].x = tmpvar_8.y;
  tmpvar_10[1].y = tmpvar_9.y;
  tmpvar_10[1].z = tmpvar_2.y;
  tmpvar_10[2].x = tmpvar_8.z;
  tmpvar_10[2].y = tmpvar_9.z;
  tmpvar_10[2].z = tmpvar_2.z;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_11;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_14;
  mediump vec4 normal_15;
  normal_15 = tmpvar_13;
  highp float vC_16;
  mediump vec3 x3_17;
  mediump vec3 x2_18;
  mediump vec3 x1_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAr, normal_15);
  x1_19.x = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAg, normal_15);
  x1_19.y = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAb, normal_15);
  x1_19.z = tmpvar_22;
  mediump vec4 tmpvar_23;
  tmpvar_23 = (normal_15.xyzz * normal_15.yzzx);
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBr, tmpvar_23);
  x2_18.x = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBg, tmpvar_23);
  x2_18.y = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBb, tmpvar_23);
  x2_18.z = tmpvar_26;
  mediump float tmpvar_27;
  tmpvar_27 = ((normal_15.x * normal_15.x) - (normal_15.y * normal_15.y));
  vC_16 = tmpvar_27;
  highp vec3 tmpvar_28;
  tmpvar_28 = (unity_SHC.xyz * vC_16);
  x3_17 = tmpvar_28;
  tmpvar_14 = ((x1_19 + x2_18) + x3_17);
  shlight_3 = tmpvar_14;
  tmpvar_6 = shlight_3;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_5;
  tmpvar_5 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), tmpvar_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  lowp float tmpvar_10;
  mediump float lightShadowDataX_11;
  highp float dist_12;
  lowp float tmpvar_13;
  tmpvar_13 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  dist_12 = tmpvar_13;
  highp float tmpvar_14;
  tmpvar_14 = _LightShadowData.x;
  lightShadowDataX_11 = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = max (float((dist_12 > (xlv_TEXCOORD4.z / xlv_TEXCOORD4.w))), lightShadowDataX_11);
  tmpvar_10 = tmpvar_15;
  highp vec3 tmpvar_16;
  tmpvar_16 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_17;
  lightDir_17 = xlv_TEXCOORD2;
  mediump vec3 viewDir_18;
  viewDir_18 = tmpvar_16;
  mediump float atten_19;
  atten_19 = tmpvar_10;
  mediump vec4 c_20;
  highp float nh_21;
  mediump float tmpvar_22;
  tmpvar_22 = max (0.0, dot (tmpvar_5, lightDir_17));
  mediump float tmpvar_23;
  tmpvar_23 = max (0.0, dot (tmpvar_5, normalize((lightDir_17 + viewDir_18))));
  nh_21 = tmpvar_23;
  mediump float tmpvar_24;
  tmpvar_24 = ((1.0 - dot (normalize(viewDir_18), tmpvar_5)) + dot (normalize(lightDir_17), tmpvar_5));
  highp vec3 tmpvar_25;
  tmpvar_25 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_22) + ((((_LightColor0.xyz * pow (nh_21, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * (atten_19 * 2.0));
  c_20.xyz = tmpvar_25;
  highp vec3 tmpvar_26;
  tmpvar_26 = (c_20.xyz + (c_20.xyz * (pow (tmpvar_24, _RimPower) * _RimColor.xyz)));
  c_20.xyz = tmpvar_26;
  c_20.w = tmpvar_9;
  c_1 = c_20;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES


#ifdef VERTEX

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (glstate_matrix_mvp * _glesVertex);
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_8;
  tmpvar_8[0] = _Object2World[0].xyz;
  tmpvar_8[1] = _Object2World[1].xyz;
  tmpvar_8[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_9;
  highp vec3 tmpvar_10;
  tmpvar_9 = tmpvar_1.xyz;
  tmpvar_10 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_11;
  tmpvar_11[0].x = tmpvar_9.x;
  tmpvar_11[0].y = tmpvar_10.x;
  tmpvar_11[0].z = tmpvar_2.x;
  tmpvar_11[1].x = tmpvar_9.y;
  tmpvar_11[1].y = tmpvar_10.y;
  tmpvar_11[1].z = tmpvar_2.y;
  tmpvar_11[2].x = tmpvar_9.z;
  tmpvar_11[2].y = tmpvar_10.z;
  tmpvar_11[2].z = tmpvar_2.z;
  highp vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_11 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_12;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_14;
  tmpvar_14.w = 1.0;
  tmpvar_14.xyz = (tmpvar_8 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_15;
  mediump vec4 normal_16;
  normal_16 = tmpvar_14;
  highp float vC_17;
  mediump vec3 x3_18;
  mediump vec3 x2_19;
  mediump vec3 x1_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAr, normal_16);
  x1_20.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAg, normal_16);
  x1_20.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHAb, normal_16);
  x1_20.z = tmpvar_23;
  mediump vec4 tmpvar_24;
  tmpvar_24 = (normal_16.xyzz * normal_16.yzzx);
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBr, tmpvar_24);
  x2_19.x = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBg, tmpvar_24);
  x2_19.y = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = dot (unity_SHBb, tmpvar_24);
  x2_19.z = tmpvar_27;
  mediump float tmpvar_28;
  tmpvar_28 = ((normal_16.x * normal_16.x) - (normal_16.y * normal_16.y));
  vC_17 = tmpvar_28;
  highp vec3 tmpvar_29;
  tmpvar_29 = (unity_SHC.xyz * vC_17);
  x3_18 = tmpvar_29;
  tmpvar_15 = ((x1_20 + x2_19) + x3_18);
  shlight_3 = tmpvar_15;
  tmpvar_6 = shlight_3;
  highp vec4 o_30;
  highp vec4 tmpvar_31;
  tmpvar_31 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_32;
  tmpvar_32.x = tmpvar_31.x;
  tmpvar_32.y = (tmpvar_31.y * _ProjectionParams.x);
  o_30.xy = (tmpvar_32 + tmpvar_31.w);
  o_30.zw = tmpvar_7.zw;
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = o_30;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 normal_5;
  normal_5.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_5.z = sqrt((1.0 - clamp (dot (normal_5.xy, normal_5.xy), 0.0, 1.0)));
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), normal_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  lowp float tmpvar_10;
  tmpvar_10 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  highp vec3 tmpvar_11;
  tmpvar_11 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_12;
  lightDir_12 = xlv_TEXCOORD2;
  mediump vec3 viewDir_13;
  viewDir_13 = tmpvar_11;
  mediump float atten_14;
  atten_14 = tmpvar_10;
  mediump vec4 c_15;
  highp float nh_16;
  mediump float tmpvar_17;
  tmpvar_17 = max (0.0, dot (normal_5, lightDir_12));
  mediump float tmpvar_18;
  tmpvar_18 = max (0.0, dot (normal_5, normalize((lightDir_12 + viewDir_13))));
  nh_16 = tmpvar_18;
  mediump float tmpvar_19;
  tmpvar_19 = ((1.0 - dot (normalize(viewDir_13), normal_5)) + dot (normalize(lightDir_12), normal_5));
  highp vec3 tmpvar_20;
  tmpvar_20 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_17) + ((((_LightColor0.xyz * pow (nh_16, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * (atten_14 * 2.0));
  c_15.xyz = tmpvar_20;
  highp vec3 tmpvar_21;
  tmpvar_21 = (c_15.xyz + (c_15.xyz * (pow (tmpvar_19, _RimPower) * _RimColor.xyz)));
  c_15.xyz = tmpvar_21;
  c_15.w = tmpvar_9;
  c_1 = c_15;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_WorldSpaceLightPos0]
Vector 15 [unity_SHAr]
Vector 16 [unity_SHAg]
Vector 17 [unity_SHAb]
Vector 18 [unity_SHBr]
Vector 19 [unity_SHBg]
Vector 20 [unity_SHBb]
Vector 21 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 22 [unity_Scale]
Vector 23 [unity_NPOTScale]
Vector 24 [_MainTex_ST]
Vector 25 [_BumpMap_ST]
"agal_vs
c26 1.0 0.5 0.0 0.0
[bc]
adaaaaaaabaaahacabaaaaoeaaaaaaaabgaaaappabaaaaaa mul r1.xyz, a1, c22.w
bcaaaaaaacaaaiacabaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r2.w, r1.xyzz, c5
bcaaaaaaaaaaabacabaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r0.x, r1.xyzz, c4
bcaaaaaaaaaaaeacabaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r0.z, r1.xyzz, c6
aaaaaaaaaaaaacacacaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r2.w
adaaaaaaabaaapacaaaaaakeacaaaaaaaaaaaacjacaaaaaa mul r1, r0.xyzz, r0.yzzx
aaaaaaaaaaaaaiacbkaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c26.x
bdaaaaaaacaaaeacaaaaaaoeacaaaaaabbaaaaoeabaaaaaa dp4 r2.z, r0, c17
bdaaaaaaacaaacacaaaaaaoeacaaaaaabaaaaaoeabaaaaaa dp4 r2.y, r0, c16
bdaaaaaaacaaabacaaaaaaoeacaaaaaaapaaaaoeabaaaaaa dp4 r2.x, r0, c15
adaaaaaaaaaaacacacaaaappacaaaaaaacaaaappacaaaaaa mul r0.y, r2.w, r2.w
bdaaaaaaadaaaeacabaaaaoeacaaaaaabeaaaaoeabaaaaaa dp4 r3.z, r1, c20
bdaaaaaaadaaacacabaaaaoeacaaaaaabdaaaaoeabaaaaaa dp4 r3.y, r1, c19
bdaaaaaaadaaabacabaaaaoeacaaaaaabcaaaaoeabaaaaaa dp4 r3.x, r1, c18
abaaaaaaabaaahacacaaaakeacaaaaaaadaaaakeacaaaaaa add r1.xyz, r2.xyzz, r3.xyzz
adaaaaaaadaaaiacaaaaaaaaacaaaaaaaaaaaaaaacaaaaaa mul r3.w, r0.x, r0.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaffacaaaaaa sub r0.x, r3.w, r0.y
adaaaaaaacaaahacaaaaaaaaacaaaaaabfaaaaoeabaaaaaa mul r2.xyz, r0.x, c21
abaaaaaaadaaahaeabaaaakeacaaaaaaacaaaakeacaaaaaa add v3.xyz, r1.xyzz, r2.xyzz
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r4.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c14, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c14, r0
aaaaaaaaabaaaiacbkaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c26.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 r0.w, a0, c3
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 r0.z, a0, c2
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaafaaahacacaaaakeacaaaaaabgaaaappabaaaaaa mul r5.xyz, r2.xyzz, c22.w
acaaaaaaacaaahacafaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r5.xyzz, a0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacaoaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c14, r1
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 r0.x, a0, c0
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 r0.y, a0, c1
adaaaaaaabaaahacaaaaaapeacaaaaaabkaaaaffabaaaaaa mul r1.xyz, r0.xyww, c26.y
adaaaaaaabaaacacabaaaaffacaaaaaaanaaaaaaabaaaaaa mul r1.y, r1.y, c13.x
abaaaaaaabaaadacabaaaafeacaaaaaaabaaaakkacaaaaaa add r1.xy, r1.xyyy, r1.z
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
adaaaaaaaeaaadaeabaaaafeacaaaaaabhaaaaoeabaaaaaa mul v4.xy, r1.xyyy, c23
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
aaaaaaaaaeaaamaeaaaaaaopacaaaaaaaaaaaaaaaaaaaaaa mov v4.zw, r0.wwzw
adaaaaaaafaaamacadaaaaeeaaaaaaaabjaaaaeeabaaaaaa mul r5.zw, a3.xyxy, c25.xyxy
abaaaaaaaaaaamaeafaaaaopacaaaaaabjaaaaoeabaaaaaa add v0.zw, r5.wwzw, c25
adaaaaaaafaaadacadaaaaoeaaaaaaaabiaaaaoeabaaaaaa mul r5.xy, a3, c24
abaaaaaaaaaaadaeafaaaafeacaaaaaabiaaaaooabaaaaaa add v0.xy, r5.xyyy, c24.zwzw
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 46 instructions, 5 temp regs, 0 temp arrays:
// ALU 26 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedfmmmjgdhgkddgeffimbjhgmlcbmbdcpkabaaaaaakeamaaaaaeaaaaaa
daaaaaaafaaeaaaacealaaaaomalaaaaebgpgodjbiaeaaaabiaeaaaaaaacpopp
jmadaaaahmaaaaaaahaaceaaaaaahiaaaaaahiaaaaaaceaaabaahiaaaaaaakaa
acaaabaaaaaaaaaaabaaaeaaacaaadaaaaaaaaaaacaaaaaaabaaafaaaaaaaaaa
acaabcaaahaaagaaaaaaaaaaadaaaaaaaeaaanaaaaaaaaaaadaaamaaadaabbaa
aaaaaaaaadaabaaaafaabeaaaaaaaaaaaaaaaaaaaaacpoppfbaaaaafbjaaapka
aaaaiadpaaaaaadpaaaaaaaaaaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaac
afaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapja
aeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaaeaaaaaeaaaaamoaadaaeeja
acaaeekaacaaoekaabaaaaacaaaaapiaafaaoekaafaaaaadabaaahiaaaaaffia
bfaaoekaaeaaaaaeabaaahiabeaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahia
bgaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiabhaaoekaaaaappiaaaaaoeia
aiaaaaadacaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaacaaoejaafaaaaad
acaaahiaabaanciaabaamjjaaeaaaaaeabaaahiaabaamjiaabaancjaacaaoeib
afaaaaadabaaahiaabaaoeiaabaappjaaiaaaaadacaaacoaabaaoeiaaaaaoeia
aiaaaaadacaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaadaaoekaafaaaaad
acaaahiaaaaaffiabfaaoekaaeaaaaaeaaaaaliabeaakekaaaaaaaiaacaakeia
aeaaaaaeaaaaahiabgaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeia
bhaaoekaaeaaaaaeaaaaahiaaaaaoeiabiaappkaaaaaoejbaiaaaaadabaaaboa
abaaoejaaaaaoeiaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoa
acaaoejaaaaaoeiaafaaaaadaaaaahiaacaaoejabiaappkaafaaaaadabaaahia
aaaaffiabcaaoekaaeaaaaaeaaaaaliabbaakekaaaaaaaiaabaakeiaaeaaaaae
aaaaahiabdaaoekaaaaakkiaaaaapeiaabaaaaacaaaaaiiabjaaaakaajaaaaad
abaaabiaagaaoekaaaaaoeiaajaaaaadabaaaciaahaaoekaaaaaoeiaajaaaaad
abaaaeiaaiaaoekaaaaaoeiaafaaaaadacaaapiaaaaacjiaaaaakeiaajaaaaad
adaaabiaajaaoekaacaaoeiaajaaaaadadaaaciaakaaoekaacaaoeiaajaaaaad
adaaaeiaalaaoekaacaaoeiaacaaaaadabaaahiaabaaoeiaadaaoeiaafaaaaad
aaaaaciaaaaaffiaaaaaffiaaeaaaaaeaaaaabiaaaaaaaiaaaaaaaiaaaaaffib
aeaaaaaeadaaahoaamaaoekaaaaaaaiaabaaoeiaafaaaaadaaaaapiaaaaaffja
aoaaoekaaeaaaaaeaaaaapiaanaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapia
apaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiabaaaoekaaaaappjaaaaaoeia
afaaaaadabaaabiaaaaaffiaaeaaaakaafaaaaadabaaaiiaabaaaaiabjaaffka
afaaaaadabaaafiaaaaapeiabjaaffkaacaaaaadaeaaadoaabaakkiaabaaomia
aeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeia
abaaaaacaeaaamoaaaaaoeiappppaaaafdeieefcmmagaaaaeaaaabaaldabaaaa
fjaaaaaeegiocaaaaaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaa
fjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaa
fpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaa
acaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaa
gfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaa
adaaaaaagfaaaaadhccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaac
afaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
abaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
acaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaaf
pccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaa
adaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaadcaaaaal
mccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaalaaaaaakgiocaaa
aaaaaaaaalaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaa
acaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaa
egacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgbpbaaaabaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaa
acaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaa
acaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaa
bdaaaaaadcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaa
abaaaaaaegacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaa
acaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaa
dcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaa
aaaaaaaaegacbaaaacaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaa
diaaaaaihcaabaaaabaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaa
diaaaaaihcaabaaaacaaaaaafgafbaaaabaaaaaaegiccaaaadaaaaaaanaaaaaa
dcaaaaaklcaabaaaabaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaabaaaaaa
egaibaaaacaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaadaaaaaaaoaaaaaa
kgakbaaaabaaaaaaegadbaaaabaaaaaadgaaaaaficaabaaaabaaaaaaabeaaaaa
aaaaiadpbbaaaaaibcaabaaaacaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaa
abaaaaaabbaaaaaiccaabaaaacaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaa
abaaaaaabbaaaaaiecaabaaaacaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaa
abaaaaaadiaaaaahpcaabaaaadaaaaaajgacbaaaabaaaaaaegakbaaaabaaaaaa
bbaaaaaibcaabaaaaeaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaadaaaaaa
bbaaaaaiccaabaaaaeaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaadaaaaaa
bbaaaaaiecaabaaaaeaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaadaaaaaa
aaaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaah
ccaabaaaabaaaaaabkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaakbcaabaaa
abaaaaaaakaabaaaabaaaaaaakaabaaaabaaaaaabkaabaiaebaaaaaaabaaaaaa
dcaaaaakhccabaaaaeaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaaabaaaaaa
egacbaaaacaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaa
abaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaa
aaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaafaaaaaakgaobaaa
aaaaaaaaaaaaaaahdccabaaaafaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaa
doaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaa
apapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaalaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaaafaaaaaa
apaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaafeeffied
epepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahaiaaaakeaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaakeaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform sampler2D _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 433
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 454
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 136
mediump vec3 ShadeSH9( in mediump vec4 normal ) {
    mediump vec3 x1;
    mediump vec3 x2;
    mediump vec3 x3;
    x1.x = dot( unity_SHAr, normal);
    #line 140
    x1.y = dot( unity_SHAg, normal);
    x1.z = dot( unity_SHAb, normal);
    mediump vec4 vB = (normal.xyzz * normal.yzzx);
    x2.x = dot( unity_SHBr, vB);
    #line 144
    x2.y = dot( unity_SHBg, vB);
    x2.z = dot( unity_SHBb, vB);
    highp float vC = ((normal.x * normal.x) - (normal.y * normal.y));
    x3 = (unity_SHC.xyz * vC);
    #line 148
    return ((x1 + x2) + x3);
}
#line 435
v2f_surf vert_surf( in appdata_full v ) {
    #line 437
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 441
    highp vec3 worldN = (mat3( _Object2World) * (v.normal * unity_Scale.w));
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    #line 445
    o.lightDir = lightDir;
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    highp vec3 shlight = ShadeSH9( vec4( worldN, 1.0));
    #line 449
    o.vlight = shlight;
    o._ShadowCoord = (unity_World2Shadow[0] * (_Object2World * v.vertex));
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out highp vec3 xlv_TEXCOORD1;
out lowp vec3 xlv_TEXCOORD2;
out lowp vec3 xlv_TEXCOORD3;
out highp vec4 xlv_TEXCOORD4;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD2 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD3 = vec3(xl_retval.vlight);
    xlv_TEXCOORD4 = vec4(xl_retval._ShadowCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform sampler2D _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 433
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 454
#line 395
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    #line 397
    mediump vec3 h = normalize((lightDir + viewDir));
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    #line 401
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    #line 405
    c.w = s.Alpha;
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 415
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 419
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 384
lowp float unitySampleShadow( in highp vec4 shadowCoord ) {
    highp float dist = textureProj( _ShadowMapTexture, shadowCoord).x;
    mediump float lightShadowDataX = _LightShadowData.x;
    #line 388
    return max( float((dist > (shadowCoord.z / shadowCoord.w))), lightShadowDataX);
}
#line 454
lowp vec4 frag_surf( in v2f_surf IN ) {
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    #line 458
    surfIN.uv_BumpMap = IN.pack0.zw;
    surfIN.viewDir = IN.viewDir;
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    #line 462
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    #line 466
    surf( surfIN, o);
    lowp float atten = unitySampleShadow( IN._ShadowCoord);
    lowp vec4 c = vec4( 0.0);
    c = LightingSimpleSpecular( o, IN.lightDir, normalize(IN.viewDir), atten);
    #line 470
    c.xyz += (o.Albedo * IN.vlight);
    c.xyz += o.Emission;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in highp vec3 xlv_TEXCOORD1;
in lowp vec3 xlv_TEXCOORD2;
in lowp vec3 xlv_TEXCOORD3;
in highp vec4 xlv_TEXCOORD4;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD1);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD2);
    xlt_IN.vlight = vec3(xlv_TEXCOORD3);
    xlt_IN._ShadowCoord = vec4(xlv_TEXCOORD4);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_ProjectionParams]
Matrix 9 [_World2Object]
Vector 16 [unity_Scale]
Vector 17 [unity_LightmapST]
Vector 18 [_MainTex_ST]
Vector 19 [_BumpMap_ST]
"!!ARBvp1.0
# 25 ALU
PARAM c[20] = { { 1, 0.5 },
		state.matrix.mvp,
		program.local[5..19] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R0.xyz, R0, vertex.attrib[14].w;
MOV R1.xyz, c[13];
MOV R1.w, c[0].x;
DP4 R0.w, vertex.position, c[4];
DP4 R2.z, R1, c[11];
DP4 R2.x, R1, c[9];
DP4 R2.y, R1, c[10];
MAD R2.xyz, R2, c[16].w, -vertex.position;
DP3 result.texcoord[1].y, R2, R0;
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[1].z, vertex.normal, R2;
DP3 result.texcoord[1].x, R2, vertex.attrib[14];
ADD result.texcoord[3].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[3].zw, R0;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[19].xyxy, c[19];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[18], c[18].zwzw;
MAD result.texcoord[2].xy, vertex.texcoord[1], c[17], c[17].zwzw;
END
# 25 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_ScreenParams]
Matrix 8 [_World2Object]
Vector 15 [unity_Scale]
Vector 16 [unity_LightmapST]
Vector 17 [_MainTex_ST]
Vector 18 [_BumpMap_ST]
"vs_2_0
; 26 ALU
def c19, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r0.xyz, r0, v1.w
mov r1.xyz, c12
mov r1.w, c19.x
dp4 r0.w, v0, c3
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c15.w, -v0
dp3 oT1.y, r2, r0
dp4 r0.z, v0, c2
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c19.y
mul r1.y, r1, c13.x
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
mad oT3.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT3.zw, r0
mad oT0.zw, v3.xyxy, c18.xyxy, c18
mad oT0.xy, v3, c17, c17.zwzw
mad oT2.xy, v4, c16, c16.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 14 [_BumpMap_ST]
Vector 13 [_MainTex_ST]
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 7 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 3 [glstate_matrix_mvp] 4
Vector 12 [unity_LightmapST]
Vector 11 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 28.00 (21 instructions), vertex: 64, texture: 0,
//   sequencer: 14,  7 GPRs, 27 threads,
// Performance (if enough threads): ~64 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacgaaaaaableaaaaaaaaaaaaaaceaaaaabmmaaaaabpeaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaaaoaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaan
aaabaaaaaaaaaanmaaaaaaaaaaaaaapiaaacaaabaaabaaaaaaaaaanmaaaaaaaa
aaaaabakaaacaaacaaabaaaaaaaaaanmaaaaaaaaaaaaabbiaaacaaahaaaeaaaa
aaaaabciaaaaaaaaaaaaabdiaaacaaaaaaabaaaaaaaaabfaaaaaaaaaaaaaabga
aaacaaadaaaeaaaaaaaaabciaaaaaaaaaaaaabhdaaacaaamaaabaaaaaaaaaanm
aaaaaaaaaaaaabieaaacaaalaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaa
fpfhgphcgmgedcepgcgkgfgdheaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaa
fpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaad
aaabaaaaaaaaaaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehj
fpemgjghgihegngbhafdfeaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
aapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabheaadbaaag
aaaaaaaaaaaaaaaaaaaadeieaaaaaaabaaaaaaafaaaaaaaiaaaaacjaaabaaaae
aaaagaafaaaadaagaaaafaahaacbfaaiaaaapafaaaachbfbaaafdcfcaaagpdfd
aaaaaablaaaababmaaaaaabgaaaaaabiaaaababjaaaababkaaaaaabfaaaababn
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
pbfffaaeaaaabcabmcaaaaaaaaaafaajaaaabcaameaaaaaaaaaagaaogabebcaa
bcaaaaaaaaaaeabkaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaaafpieaaa
aaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmiaaaaaaaa
afpibaaaaaaaacdpaaaaaaaamiapaaaaaabliiaakbadagaamiapaaaaaamgnapi
kladafaamiapaaaaaalbdepikladaeaamiapaaagaagmnajekladadaamiapiado
aananaaaocagagaamiahaaaaaamamgmaalajaaakmiahaaafaalogfaaobacaeaa
miahaaaaaalelbleclaiaaaamiahaaaaaamagmleclahaaaamiahaaafabgfloma
olacaeafmiahaaadabmablmaklaaaladmiahaaaaaamagmaakbagppaamiamiaad
aanlnlaaocagagaamiabiaabaaloloaapaadaeaakiihaaaeaamablebmbafaeab
miaciaabaaloloaapaaeadaamiaeiaabaaloloaapaadacaamiadiaacaabklabk
ilabamammiadiaaaaalalabkilabananmiamiaaaaakmkmagilabaoaomiadiaad
aamgbkbiklaaacaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Vector 464 [unity_LightmapST]
Vector 463 [_MainTex_ST]
Vector 462 [_BumpMap_ST]
"sce_vp_rsx // 24 instructions using 4 registers
[Configuration]
8
0000001843050400
[Defaults]
1
461 1
3f000000
[Microcode]
384
00011c6c00400e0c0106c0836041dffc00019c6c005d300c0186c0836041dffc
401f9c6c011ce800810040d560607f9c401f9c6c011cf808010400d740619f9c
401f9c6c011d0908010400d740619fa400009c6c01d0300d8106c0c360403ffc
00009c6c01d0200d8106c0c360405ffc00009c6c01d0100d8106c0c360409ffc
00009c6c01d0000d8106c0c360411ffc00001c6c0190a00c0686c0c360405ffc
00001c6c0190900c0686c0c360409ffc00001c6c0190800c0686c0c360411ffc
00019c6c00800243011842436041dffc00011c6c010002308121826301a1dffc
401f9c6c0040000d8286c0836041ff80401f9c6c004000558286c08360407fa8
00001c6c011d100c00bfc0e30041dffc00009c6c009cd00e028000c36041dffc
00009c6c009d202a828000c360409ffc401f9c6c00c000080286c09540a19fa8
00009c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106004360405fa0
401f9c6c01400e0c0086008360411fa0401f9c6c0140000c0086014360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 208 // 208 used size, 10 vars
Vector 160 [unity_LightmapST] 4
Vector 176 [_MainTex_ST] 4
Vector 192 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 24 instructions, 3 temp regs, 0 temp arrays:
// ALU 11 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefieceddbhdegiogcjlhjfoajllidhebfjnidhjabaaaaaaiaafaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
adamaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcoeadaaaaeaaaabaa
pjaaaaaafjaaaaaeegiocaaaaaaaaaaaanaaaaaafjaaaaaeegiocaaaabaaaaaa
agaaaaaafjaaaaaeegiocaaaacaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaa
adaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaa
gfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaaddccabaaa
adaaaaaagfaaaaadpccabaaaaeaaaaaagiaaaaacadaaaaaadiaaaaaipcaabaaa
aaaaaaaafgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaakgbkbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaadaaaaaa
pgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaa
alaaaaaaogikcaaaaaaaaaaaalaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaa
adaaaaaaagiecaaaaaaaaaaaamaaaaaakgiocaaaaaaaaaaaamaaaaaadiaaaaah
hcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
abaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaabaaaaaa
diaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaadiaaaaaj
hcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaacaaaaaabbaaaaaa
dcaaaaalhcaabaaaacaaaaaaegiccaaaacaaaaaabaaaaaaaagiacaaaabaaaaaa
aeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaacaaaaaa
bcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaaaaaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaaegiccaaaacaaaaaabdaaaaaadcaaaaalhcaabaaa
acaaaaaaegacbaaaacaaaaaapgipcaaaacaaaaaabeaaaaaaegbcbaiaebaaaaaa
aaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaah
eccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadcaaaaaldccabaaa
adaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaa
akaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaaeaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaaeaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES


#ifdef VERTEX

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  tmpvar_4 = tmpvar_1.xyz;
  tmpvar_5 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_6;
  tmpvar_6[0].x = tmpvar_4.x;
  tmpvar_6[0].y = tmpvar_5.x;
  tmpvar_6[0].z = tmpvar_2.x;
  tmpvar_6[1].x = tmpvar_4.y;
  tmpvar_6[1].y = tmpvar_5.y;
  tmpvar_6[1].z = tmpvar_2.y;
  tmpvar_6[2].x = tmpvar_4.z;
  tmpvar_6[2].y = tmpvar_5.z;
  tmpvar_6[2].z = tmpvar_2.z;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = _WorldSpaceCameraPos;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_7).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec3 tmpvar_4;
  tmpvar_4 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  highp float tmpvar_5;
  tmpvar_5 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), tmpvar_4), 0.0, 1.0));
  rim_3 = tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_6 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_6;
  lowp float tmpvar_7;
  mediump float lightShadowDataX_8;
  highp float dist_9;
  lowp float tmpvar_10;
  tmpvar_10 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD3).x;
  dist_9 = tmpvar_10;
  highp float tmpvar_11;
  tmpvar_11 = _LightShadowData.x;
  lightShadowDataX_8 = tmpvar_11;
  highp float tmpvar_12;
  tmpvar_12 = max (float((dist_9 > (xlv_TEXCOORD3.z / xlv_TEXCOORD3.w))), lightShadowDataX_8);
  tmpvar_7 = tmpvar_12;
  c_1.xyz = (texture2D (_MainTex, xlv_TEXCOORD0.xy).xyz * min ((2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD2).xyz), vec3((tmpvar_7 * 2.0))));
  c_1.w = texture2D (_MainTex, xlv_TEXCOORD0.xy).w;
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES


#ifdef VERTEX

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (glstate_matrix_mvp * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  highp vec4 o_9;
  highp vec4 tmpvar_10;
  tmpvar_10 = (tmpvar_4 * 0.5);
  highp vec2 tmpvar_11;
  tmpvar_11.x = tmpvar_10.x;
  tmpvar_11.y = (tmpvar_10.y * _ProjectionParams.x);
  o_9.xy = (tmpvar_11 + tmpvar_10.w);
  o_9.zw = tmpvar_4.zw;
  gl_Position = tmpvar_4;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = o_9;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec3 normal_4;
  normal_4.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_4.z = sqrt((1.0 - clamp (dot (normal_4.xy, normal_4.xy), 0.0, 1.0)));
  highp float tmpvar_5;
  tmpvar_5 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), normal_4), 0.0, 1.0));
  rim_3 = tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_6 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD3);
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (unity_Lightmap, xlv_TEXCOORD2);
  lowp vec3 tmpvar_9;
  tmpvar_9 = ((8.0 * tmpvar_8.w) * tmpvar_8.xyz);
  c_1.xyz = (texture2D (_MainTex, xlv_TEXCOORD0.xy).xyz * max (min (tmpvar_9, ((tmpvar_7.x * 2.0) * tmpvar_8.xyz)), (tmpvar_9 * tmpvar_7.x)));
  c_1.w = texture2D (_MainTex, xlv_TEXCOORD0.xy).w;
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Matrix 8 [_World2Object]
Vector 14 [unity_Scale]
Vector 15 [unity_NPOTScale]
Vector 16 [unity_LightmapST]
Vector 17 [_MainTex_ST]
Vector 18 [_BumpMap_ST]
"agal_vs
c19 1.0 0.5 0.0 0.0
[bc]
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaacaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r2.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r2.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r0.xyz, r0.xyzz, a5.w
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
aaaaaaaaabaaaiacbdaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c19.x
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaadaaahacacaaaakeacaaaaaaaoaaaappabaaaaaa mul r3.xyz, r2.xyzz, c14.w
acaaaaaaabaaahacadaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r1.xyz, r3.xyzz, a0
bcaaaaaaabaaacaeabaaaakeacaaaaaaaaaaaakeacaaaaaa dp3 v1.y, r1.xyzz, r0.xyzz
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 r0.w, a0, c3
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 r0.z, a0, c2
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaabaaaakeacaaaaaa dp3 v1.z, a1, r1.xyzz
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 r0.x, a0, c0
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 r0.y, a0, c1
adaaaaaaacaaahacaaaaaapeacaaaaaabdaaaaffabaaaaaa mul r2.xyz, r0.xyww, c19.y
bcaaaaaaabaaabaeabaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r1.xyzz, a5
adaaaaaaabaaacacacaaaaffacaaaaaaanaaaaaaabaaaaaa mul r1.y, r2.y, c13.x
aaaaaaaaabaaabacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r2.x
abaaaaaaabaaadacabaaaafeacaaaaaaacaaaakkacaaaaaa add r1.xy, r1.xyyy, r2.z
adaaaaaaadaaadaeabaaaafeacaaaaaaapaaaaoeabaaaaaa mul v3.xy, r1.xyyy, c15
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
aaaaaaaaadaaamaeaaaaaaopacaaaaaaaaaaaaaaaaaaaaaa mov v3.zw, r0.wwzw
adaaaaaaadaaamacadaaaaeeaaaaaaaabcaaaaeeabaaaaaa mul r3.zw, a3.xyxy, c18.xyxy
abaaaaaaaaaaamaeadaaaaopacaaaaaabcaaaaoeabaaaaaa add v0.zw, r3.wwzw, c18
adaaaaaaadaaadacadaaaaoeaaaaaaaabbaaaaoeabaaaaaa mul r3.xy, a3, c17
abaaaaaaaaaaadaeadaaaafeacaaaaaabbaaaaooabaaaaaa add v0.xy, r3.xyyy, c17.zwzw
adaaaaaaadaaadacaeaaaaoeaaaaaaaabaaaaaoeabaaaaaa mul r3.xy, a4, c16
abaaaaaaacaaadaeadaaaafeacaaaaaabaaaaaooabaaaaaa add v2.xy, r3.xyyy, c16.zwzw
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.zw, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 208 // 208 used size, 10 vars
Vector 160 [unity_LightmapST] 4
Vector 176 [_MainTex_ST] 4
Vector 192 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 24 instructions, 3 temp regs, 0 temp arrays:
// ALU 11 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedkllclefkfecbapaakjdpapefepkpiipkabaaaaaaamaiaaaaaeaaaaaa
daaaaaaaliacaaaakeagaaaagmahaaaaebgpgodjiaacaaaaiaacaaaaaaacpopp
ciacaaaafiaaaaaaaeaaceaaaaaafeaaaaaafeaaaaaaceaaabaafeaaaaaaakaa
adaaabaaaaaaaaaaabaaaeaaacaaaeaaaaaaaaaaacaaaaaaaeaaagaaaaaaaaaa
acaabaaaafaaakaaaaaaaaaaaaaaaaaaaaacpoppfbaaaaafapaaapkaaaaaaadp
aaaaaaaaaaaaaaaaaaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjabpaaaaac
afaaaeiaaeaaapjaaeaaaaaeaaaaadoaadaaoejaacaaoekaacaaookaaeaaaaae
aaaaamoaadaaeejaadaaeekaadaaoekaaeaaaaaeacaaadoaaeaaoejaabaaoeka
abaaookaabaaaaacaaaaahiaaeaaoekaafaaaaadabaaahiaaaaaffiaalaaoeka
aeaaaaaeaaaaaliaakaakekaaaaaaaiaabaakeiaaeaaaaaeaaaaahiaamaaoeka
aaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeiaanaaoekaaeaaaaaeaaaaahia
aaaaoeiaaoaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaaaaaoeiaabaaaaac
abaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaaeabaaahia
acaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappjaaiaaaaad
abaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeiaafaaaaad
aaaaapiaaaaaffjaahaaoekaaeaaaaaeaaaaapiaagaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaaiaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaajaaoeka
aaaappjaaaaaoeiaafaaaaadabaaabiaaaaaffiaafaaaakaafaaaaadabaaaiia
abaaaaiaapaaaakaafaaaaadabaaafiaaaaapeiaapaaaakaacaaaaadadaaadoa
abaakkiaabaaomiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaac
aaaaammaaaaaoeiaabaaaaacadaaamoaaaaaoeiappppaaaafdeieefcoeadaaaa
eaaaabaapjaaaaaafjaaaaaeegiocaaaaaaaaaaaanaaaaaafjaaaaaeegiocaaa
abaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaabfaaaaaafpaaaaadpcbabaaa
aaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaad
dcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
dccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagiaaaaacadaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaa
aaaaaaaaalaaaaaaogikcaaaaaaaaaaaalaaaaaadcaaaaalmccabaaaabaaaaaa
agbebaaaadaaaaaaagiecaaaaaaaaaaaamaaaaaakgiocaaaaaaaaaaaamaaaaaa
diaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
abaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaacaaaaaa
bbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaacaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
acaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaaaaaaaaai
hcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaacaaaaaabdaaaaaadcaaaaal
hcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaacaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadcaaaaal
dccabaaaadaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaa
aaaaaaaaakaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaa
abaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaa
aaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaaeaaaaaakgaobaaa
aaaaaaaaaaaaaaahdccabaaaaeaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaa
doaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaa
apapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaalaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaaafaaaaaa
apaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaafeeffied
epepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaadamaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    highp vec2 lmap;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform sampler2D _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 432
uniform highp vec4 unity_LightmapST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
uniform sampler2D unity_Lightmap;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 435
v2f_surf vert_surf( in appdata_full v ) {
    #line 437
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 441
    o.lmap.xy = ((v.texcoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
    highp vec3 worldN = (mat3( _Object2World) * (v.normal * unity_Scale.w));
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    #line 445
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    o._ShadowCoord = (unity_World2Shadow[0] * (_Object2World * v.vertex));
    #line 450
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out highp vec3 xlv_TEXCOORD1;
out highp vec2 xlv_TEXCOORD2;
out highp vec4 xlv_TEXCOORD3;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD2 = vec2(xl_retval.lmap);
    xlv_TEXCOORD3 = vec4(xl_retval._ShadowCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    highp vec2 lmap;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform sampler2D _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 432
uniform highp vec4 unity_LightmapST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
uniform sampler2D unity_Lightmap;
#line 176
lowp vec3 DecodeLightmap( in lowp vec4 color ) {
    #line 178
    return (2.0 * color.xyz);
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 415
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 419
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 384
lowp float unitySampleShadow( in highp vec4 shadowCoord ) {
    highp float dist = textureProj( _ShadowMapTexture, shadowCoord).x;
    mediump float lightShadowDataX = _LightShadowData.x;
    #line 388
    return max( float((dist > (shadowCoord.z / shadowCoord.w))), lightShadowDataX);
}
#line 453
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 455
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    surfIN.viewDir = IN.viewDir;
    #line 459
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    #line 463
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp float atten = unitySampleShadow( IN._ShadowCoord);
    #line 467
    lowp vec4 c = vec4( 0.0);
    lowp vec4 lmtex = texture( unity_Lightmap, IN.lmap.xy);
    lowp vec3 lm = DecodeLightmap( lmtex);
    c.xyz += (o.Albedo * min( lm, vec3( (atten * 2.0))));
    #line 471
    c.w = o.Alpha;
    c.xyz += o.Emission;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in highp vec3 xlv_TEXCOORD1;
in highp vec2 xlv_TEXCOORD2;
in highp vec4 xlv_TEXCOORD3;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD1);
    xlt_IN.lmap = vec2(xlv_TEXCOORD2);
    xlt_IN._ShadowCoord = vec4(xlv_TEXCOORD3);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_WorldSpaceLightPos0]
Vector 15 [unity_4LightPosX0]
Vector 16 [unity_4LightPosY0]
Vector 17 [unity_4LightPosZ0]
Vector 18 [unity_4LightAtten0]
Vector 19 [unity_LightColor0]
Vector 20 [unity_LightColor1]
Vector 21 [unity_LightColor2]
Vector 22 [unity_LightColor3]
Vector 23 [unity_SHAr]
Vector 24 [unity_SHAg]
Vector 25 [unity_SHAb]
Vector 26 [unity_SHBr]
Vector 27 [unity_SHBg]
Vector 28 [unity_SHBb]
Vector 29 [unity_SHC]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 30 [unity_Scale]
Vector 31 [_MainTex_ST]
Vector 32 [_BumpMap_ST]
"!!ARBvp1.0
# 75 ALU
PARAM c[33] = { { 1, 0 },
		state.matrix.mvp,
		program.local[5..32] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
MUL R3.xyz, vertex.normal, c[30].w;
DP4 R0.x, vertex.position, c[6];
ADD R1, -R0.x, c[16];
DP3 R3.w, R3, c[6];
DP3 R4.x, R3, c[5];
DP3 R3.x, R3, c[7];
MUL R2, R3.w, R1;
DP4 R0.x, vertex.position, c[5];
ADD R0, -R0.x, c[15];
MUL R1, R1, R1;
MOV R4.z, R3.x;
MAD R2, R4.x, R0, R2;
MOV R4.w, c[0].x;
DP4 R4.y, vertex.position, c[7];
MAD R1, R0, R0, R1;
ADD R0, -R4.y, c[17];
MAD R1, R0, R0, R1;
MAD R0, R3.x, R0, R2;
MUL R2, R1, c[18];
MOV R4.y, R3.w;
RSQ R1.x, R1.x;
RSQ R1.y, R1.y;
RSQ R1.w, R1.w;
RSQ R1.z, R1.z;
MUL R0, R0, R1;
ADD R1, R2, c[0].x;
RCP R1.x, R1.x;
RCP R1.y, R1.y;
RCP R1.w, R1.w;
RCP R1.z, R1.z;
MAX R0, R0, c[0].y;
MUL R0, R0, R1;
MUL R1.xyz, R0.y, c[20];
MAD R1.xyz, R0.x, c[19], R1;
MAD R0.xyz, R0.z, c[21], R1;
MAD R1.xyz, R0.w, c[22], R0;
MUL R0, R4.xyzz, R4.yzzx;
DP4 R3.z, R0, c[28];
DP4 R3.y, R0, c[27];
DP4 R3.x, R0, c[26];
MUL R1.w, R3, R3;
MAD R0.x, R4, R4, -R1.w;
MOV R0.w, c[0].x;
DP4 R2.z, R4, c[25];
DP4 R2.y, R4, c[24];
DP4 R2.x, R4, c[23];
ADD R2.xyz, R2, R3;
MUL R3.xyz, R0.x, c[29];
ADD R3.xyz, R2, R3;
MOV R0.xyz, vertex.attrib[14];
MUL R2.xyz, vertex.normal.zxyw, R0.yzxw;
ADD result.texcoord[3].xyz, R3, R1;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R2;
MOV R0.xyz, c[13];
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[30].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[14];
DP4 R3.z, R1, c[11];
DP4 R3.x, R1, c[9];
DP4 R3.y, R1, c[10];
DP3 result.texcoord[1].y, R0, R2;
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[32].xyxy, c[32];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[31], c[31].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 75 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_WorldSpaceLightPos0]
Vector 14 [unity_4LightPosX0]
Vector 15 [unity_4LightPosY0]
Vector 16 [unity_4LightPosZ0]
Vector 17 [unity_4LightAtten0]
Vector 18 [unity_LightColor0]
Vector 19 [unity_LightColor1]
Vector 20 [unity_LightColor2]
Vector 21 [unity_LightColor3]
Vector 22 [unity_SHAr]
Vector 23 [unity_SHAg]
Vector 24 [unity_SHAb]
Vector 25 [unity_SHBr]
Vector 26 [unity_SHBg]
Vector 27 [unity_SHBb]
Vector 28 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 29 [unity_Scale]
Vector 30 [_MainTex_ST]
Vector 31 [_BumpMap_ST]
"vs_2_0
; 78 ALU
def c32, 1.00000000, 0.00000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r3.xyz, v2, c29.w
dp4 r0.x, v0, c5
add r1, -r0.x, c15
dp3 r3.w, r3, c5
dp3 r4.x, r3, c4
dp3 r3.x, r3, c6
mul r2, r3.w, r1
dp4 r0.x, v0, c4
add r0, -r0.x, c14
mul r1, r1, r1
mov r4.z, r3.x
mad r2, r4.x, r0, r2
mov r4.w, c32.x
dp4 r4.y, v0, c6
mad r1, r0, r0, r1
add r0, -r4.y, c16
mad r1, r0, r0, r1
mad r0, r3.x, r0, r2
mul r2, r1, c17
mov r4.y, r3.w
rsq r1.x, r1.x
rsq r1.y, r1.y
rsq r1.w, r1.w
rsq r1.z, r1.z
mul r0, r0, r1
add r1, r2, c32.x
dp4 r2.z, r4, c24
dp4 r2.y, r4, c23
dp4 r2.x, r4, c22
rcp r1.x, r1.x
rcp r1.y, r1.y
rcp r1.w, r1.w
rcp r1.z, r1.z
max r0, r0, c32.y
mul r0, r0, r1
mul r1.xyz, r0.y, c19
mad r1.xyz, r0.x, c18, r1
mad r0.xyz, r0.z, c20, r1
mad r1.xyz, r0.w, c21, r0
mul r0, r4.xyzz, r4.yzzx
mul r1.w, r3, r3
dp4 r3.z, r0, c27
dp4 r3.y, r0, c26
dp4 r3.x, r0, c25
mad r1.w, r4.x, r4.x, -r1
mul r0.xyz, r1.w, c28
add r2.xyz, r2, r3
add r2.xyz, r2, r0
add oT3.xyz, r2, r1
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c13, r0
mov r0, c9
mov r1.w, c32.x
mov r1.xyz, c12
dp4 r4.y, c13, r0
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c29.w, -v0
mov r1, c8
dp4 r4.x, c13, r1
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
mad oT0.zw, v3.xyxy, c31.xyxy, c31
mad oT0.xy, v3, c30, c30.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 31 [_BumpMap_ST]
Vector 30 [_MainTex_ST]
Matrix 21 [_Object2World] 4
Matrix 25 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 17 [glstate_matrix_mvp] 4
Vector 5 [unity_4LightAtten0]
Vector 2 [unity_4LightPosX0]
Vector 3 [unity_4LightPosY0]
Vector 4 [unity_4LightPosZ0]
Vector 6 [unity_LightColor0]
Vector 7 [unity_LightColor1]
Vector 8 [unity_LightColor2]
Vector 9 [unity_LightColor3]
Vector 12 [unity_SHAb]
Vector 11 [unity_SHAg]
Vector 10 [unity_SHAr]
Vector 15 [unity_SHBb]
Vector 14 [unity_SHBg]
Vector 13 [unity_SHBr]
Vector 16 [unity_SHC]
Vector 29 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 90.67 (68 instructions), vertex: 32, texture: 0,
//   sequencer: 30,  10 GPRs, 18 threads,
// Performance (if enough threads): ~90 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadoeaaaaaeamaaaaaaaaaaaaaaceaaaaadfaaaaaadhiaaaaaaaa
aaaaaaaaaaaaadciaaaaaabmaaaaadblpppoadaaaaaaaabeaaaaaabmaaaaaaaa
aaaaadbeaaaaabkmaaacaabpaaabaaaaaaaaabliaaaaaaaaaaaaabmiaaacaabo
aaabaaaaaaaaabliaaaaaaaaaaaaabneaaacaabfaaaeaaaaaaaaaboeaaaaaaaa
aaaaabpeaaacaabjaaaeaaaaaaaaaboeaaaaaaaaaaaaacacaaacaaaaaaabaaaa
aaaaacbiaaaaaaaaaaaaacciaaacaaabaaabaaaaaaaaabliaaaaaaaaaaaaacdn
aaacaabbaaaeaaaaaaaaaboeaaaaaaaaaaaaacfaaaacaaafaaabaaaaaaaaabli
aaaaaaaaaaaaacgdaaacaaacaaabaaaaaaaaabliaaaaaaaaaaaaachfaaacaaad
aaabaaaaaaaaabliaaaaaaaaaaaaacihaaacaaaeaaabaaaaaaaaabliaaaaaaaa
aaaaacjjaaacaaagaaaeaaaaaaaaackmaaaaaaaaaaaaaclmaaacaaamaaabaaaa
aaaaabliaaaaaaaaaaaaacmhaaacaaalaaabaaaaaaaaabliaaaaaaaaaaaaacnc
aaacaaakaaabaaaaaaaaabliaaaaaaaaaaaaacnnaaacaaapaaabaaaaaaaaabli
aaaaaaaaaaaaacoiaaacaaaoaaabaaaaaaaaabliaaaaaaaaaaaaacpdaaacaaan
aaabaaaaaaaaabliaaaaaaaaaaaaacpoaaacaabaaaabaaaaaaaaabliaaaaaaaa
aaaaadaiaaacaabnaaabaaaaaaaaabliaaaaaaaafpechfgnhaengbhafpfdfeaa
aaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpepgcgk
gfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphc
gmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaakl
aaabaaadaaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihe
fagphddaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpdeem
gjghgiheebhehegfgodaaahfgogjhehjfpdeemgjghgihefagphdfidaaahfgogj
hehjfpdeemgjghgihefagphdfjdaaahfgogjhehjfpdeemgjghgihefagphdfkda
aahfgogjhehjfpemgjghgiheedgpgmgphcaaklklaaabaaadaaabaaaeaaaeaaaa
aaaaaaaahfgogjhehjfpfdeiebgcaahfgogjhehjfpfdeiebghaahfgogjhehjfp
fdeiebhcaahfgogjhehjfpfdeiecgcaahfgogjhehjfpfdeiecghaahfgogjhehj
fpfdeiechcaahfgogjhehjfpfdeiedaahfgogjhehjfpfdgdgbgmgfaahghdfpdd
fpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeaapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaadmm
aadbaaajaaaaaaaaaaaaaaaaaaaadeieaaaaaaabaaaaaaaeaaaaaaajaaaaacja
aabaaaaiaaaagaajaaaadaakaadafaalaaaapafaaaachbfbaaafhcfcaaaihdfd
aaaaaacjaaaabackaaaaaacdaaaaaaceaaaabacfaaaaaacgaaaaaachaaaabaci
aaaabaepaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaaaaaaaaaaaaaaaaaa
aaaaaaaapaffeaaiaaaabcaamcaaaaaaaaaaeaamaaaabcaameaaaaaaaaaagaba
gabgbcaabcaaaaaaaaaagabmgaccbcaabcaaaaaaaaaagacigacobcaabcaaaaaa
aaaagadegadkbcaabcaaaaaaaaaagaeagaegbcaabcaaaaaaaaaaeaemaaaaccaa
aaaaaaaaafpiiaaaaaaaaanbaaaaaaaaafpifaaaaaaaagiiaaaaaaaaafpicaaa
aaaaaoiiaaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaabaamgiiaakbaibeaa
miapaaabaalbiiaaklaibdabmiapaaabaagmdejeklaibcabmiapiadoaablaade
klaibbabmiahaaabaamamgmaalblaabmmiahaaadaaleblaacbbmabaamiahaaad
aamamgleclblabadmiahaaaeaalogfaaobacafaamiahaaahaalelbleclbkaaab
miahaaabaagfblaakbacbnaamiahaaagaamgleaakbaibiaamiahaaajaalbmale
klaibhagmiahaaagaalbleaakbabbhaamiahaaahaamagmleclbjaaahmiahaaae
abgflomaolacafaemiahaaadaalelbleclbkabadmiahaaadaamagmleclbjabad
miahaaaeaamablaaobaeafaamiahaaahabmabllpklahbnaimiahaaagaagmlema
klabbgagmiahaaaiaagmleleklaibgajmialaaabaabllemaklaibfaimiahaaag
aamglemaklabbfagmiabiaabaaloloaapaahafaamiaciaabaaloloaapaaeahaa
miaeiaabaaloloaapaahacaamiabiaacaaloloaapaadafaamiaciaacaaloloaa
paaeadaamiaeiaacaaloloaapaadacaamiadiaaaaalalabkilaabobomiamiaaa
aakmkmagilaabpbpceipagaaaalehcgmobagagiaaibpadafaegmaagmkaabacag
aicpadacaelbaamgkaabaeagbeabaaaeabdoanblgpakagabaebcahaeaadoangm
epalagadbeaeaaaeabdoanblgpamagabaecbahabaakhkhlbipaaanadbeacaaab
abkhkhblkpaaaoabaeeeahabaakhkhmgipaaapadbeapaaaaabpipiblobacacab
aeipahacaapilbblmbacagadmiapaaaaaajejepiolahahaamiapaaacaajemgpi
olahagacmiapaaacaajegmaaolafagacmiapaaaaaaaaaapiolafafaageihabab
aalologboaaeabadmiahaaabaabllemnklabbaabmiapaaaeaapipigmilaaafpp
fibaaaaaaaaaaagmocaaaaiaficaaaaaaaaaaalbocaaaaiafieaaaaaaaaaaamg
ocaaaaiafiiaaaaaaaaaaablocaaaaiamiapaaaaaapiaaaaobacaaaaemipaaad
aapilbmgkcaappaeemecacaaaamgblgmobadaaaeemciacacaagmmgblobadacae
embbaaacaabllblbobadacaemiaeaaaaaalbgmaaobadaaaakibhacaeaalmmaec
ibacaiajkiciacaeaamgblicmbaeadajkieoacafaabgpmmaibacagajbeahaaaa
aabbmalbkbaaahafambiafaaaamgmggmobaaadadbeahaaaaaabebamgoaafaaac
amihacaaaamabalboaaaaeadmiahaaaaaamabaaaoaaaacaamiahiaadaalemaaa
oaabaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Vector 465 [unity_4LightPosX0]
Vector 464 [unity_4LightPosY0]
Vector 463 [unity_4LightPosZ0]
Vector 462 [unity_4LightAtten0]
Vector 461 [unity_LightColor0]
Vector 460 [unity_LightColor1]
Vector 459 [unity_LightColor2]
Vector 458 [unity_LightColor3]
Vector 457 [unity_SHAr]
Vector 456 [unity_SHAg]
Vector 455 [unity_SHAb]
Vector 454 [unity_SHBr]
Vector 453 [unity_SHBg]
Vector 452 [unity_SHBb]
Vector 451 [unity_SHC]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 450 [unity_Scale]
Vector 449 [_MainTex_ST]
Vector 448 [_BumpMap_ST]
"sce_vp_rsx // 64 instructions using 9 registers
[Configuration]
8
0000004041050900
[Defaults]
1
447 2
000000003f800000
[Microcode]
1024
00011c6c005d200d8186c0836041fffc00031c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc00009c6c009c220c013fc0c36041dffc
401f9c6c011c0800810040d560607f9c401f9c6c011c1808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00019c6c01d0500d8106c0c360411ffc00009c6c01d0400d8106c0c360403ffc
00001c6c01d0600d8106c0c360403ffc00029c6c01d0a00d8486c0c360405ffc
00029c6c01d0900d8486c0c360409ffc00029c6c01d0800d8486c0c360411ffc
00021c6c0150400c028600c360411ffc00021c6c0150600c028600c360403ffc
00021c6c0150500c028600c360409ffc00011c6c0190a00c0086c0c360405ffc
00011c6c0190900c0086c0c360409ffc00011c6c0190800c0086c0c360411ffc
00001c6c00dcf00d8186c0bfe021fffc00009c6c00dd100d8186c0bfe0a1fffc
00019c6c00dd000d8186c0a001a1fffc00039c6c00800243011846436041dffc
00039c6c010002308121866303a1dffc00031c6c011c200c04bfc0e30041dffc
401f9c6c0140020c0106054360405fa400011c6c0080002a8886c3436041fffc
00019c6c0080000d8686c3436041fffc00029c6c0080002a8895444360403ffc
00021c6c0040007f8886c08360405ffc00011c6c010000000886c1436121fffc
00009c6c0100000d8286c14361a1fffc00041c6c019c700c0886c0c360405ffc
00041c6c019c800c0886c0c360409ffc00041c6c019c900c0886c0c360411ffc
00029c6c010000000880047fe2a03ffc00019c6c0080000d089a04436041fffc
00011c6c0100007f8886c0436121fffc00001c6c0100000d8086c04360a1fffc
00009c6c01dc400d8686c0c360405ffc00009c6c01dc500d8686c0c360409ffc
00009c6c01dc600d8686c0c360411ffc00009c6c00c0000c1086c08300a1dffc
00019c6c009c307f8a8600c36041dffc00019c6c00c0000c0686c08300a1dffc
401f9c6c21400e0c01060540003100a400039c6c20800e0c0ebfc08aa029c0fc
00021c6c209ce00d8086c0d54025e0fc00021c6c00dbf02a8186c0836221fffc
401f9c6c2140020c0106065fe02240a0401f9c6c11400e0c0c86008002310020
401f9c6c1140000c0e86054aa2288024401f9c6c1140000c0c86075542248020
00009c6c1080000d8486c15fe223e07c00009c6c029bf00d828000c36041fffc
00001c6c0080000d8286c0436041fffc00009c6c009cc02a808600c36041dffc
00009c6c011cd000008600c300a1dffc00001c6c011cb055008600c300a1dffc
00001c6c011ca07f808600c30021dffc401f9c6c00c0000c0686c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 128 used size, 8 vars
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 32 [unity_4LightPosX0] 4
Vector 48 [unity_4LightPosY0] 4
Vector 64 [unity_4LightPosZ0] 4
Vector 80 [unity_4LightAtten0] 4
Vector 96 [unity_LightColor0] 4
Vector 112 [unity_LightColor1] 4
Vector 128 [unity_LightColor2] 4
Vector 144 [unity_LightColor3] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 65 instructions, 6 temp regs, 0 temp arrays:
// ALU 36 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedehgdndojeegeiogiomhlcmnhcgecjegiabaaaaaacaalaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcieajaaaaeaaaabaa
gbacaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacagaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaahaaaaaa
kgiocaaaaaaaaaaaahaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadpdiaaaaaihcaabaaa
abaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaa
acaaaaaafgafbaaaabaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaa
abaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaabaaaaaaegaibaaaacaaaaaa
dcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaabaaaaaa
egadbaaaabaaaaaabbaaaaaibcaabaaaabaaaaaaegiocaaaacaaaaaabcaaaaaa
egaobaaaaaaaaaaabbaaaaaiccaabaaaabaaaaaaegiocaaaacaaaaaabdaaaaaa
egaobaaaaaaaaaaabbaaaaaiecaabaaaabaaaaaaegiocaaaacaaaaaabeaaaaaa
egaobaaaaaaaaaaadiaaaaahpcaabaaaacaaaaaajgacbaaaaaaaaaaaegakbaaa
aaaaaaaabbaaaaaibcaabaaaadaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaa
acaaaaaabbaaaaaiccaabaaaadaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaa
acaaaaaabbaaaaaiecaabaaaadaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaa
acaaaaaaaaaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaa
diaaaaahicaabaaaaaaaaaaabkaabaaaaaaaaaaabkaabaaaaaaaaaaadcaaaaak
icaabaaaaaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaadkaabaiaebaaaaaa
aaaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaa
aaaaaaaaegacbaaaabaaaaaadiaaaaaihcaabaaaacaaaaaafgbfbaaaaaaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaadaaaaaa
amaaaaaaagbabaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaaacaaaaaa
egiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaacaaaaaadcaaaaak
hcaabaaaacaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaa
acaaaaaaaaaaaaajpcaabaaaadaaaaaafgafbaiaebaaaaaaacaaaaaaegiocaaa
acaaaaaaadaaaaaadiaaaaahpcaabaaaaeaaaaaafgafbaaaaaaaaaaaegaobaaa
adaaaaaadiaaaaahpcaabaaaadaaaaaaegaobaaaadaaaaaaegaobaaaadaaaaaa
aaaaaaajpcaabaaaafaaaaaaagaabaiaebaaaaaaacaaaaaaegiocaaaacaaaaaa
acaaaaaaaaaaaaajpcaabaaaacaaaaaakgakbaiaebaaaaaaacaaaaaaegiocaaa
acaaaaaaaeaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaaafaaaaaaagaabaaa
aaaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaaaaaaaaaaegaobaaaacaaaaaa
kgakbaaaaaaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaaadaaaaaaegaobaaa
afaaaaaaegaobaaaafaaaaaaegaobaaaadaaaaaadcaaaaajpcaabaaaacaaaaaa
egaobaaaacaaaaaaegaobaaaacaaaaaaegaobaaaadaaaaaaeeaaaaafpcaabaaa
adaaaaaaegaobaaaacaaaaaadcaaaaanpcaabaaaacaaaaaaegaobaaaacaaaaaa
egiocaaaacaaaaaaafaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadp
aoaaaaakpcaabaaaacaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadp
egaobaaaacaaaaaadiaaaaahpcaabaaaaaaaaaaaegaobaaaaaaaaaaaegaobaaa
adaaaaaadeaaaaakpcaabaaaaaaaaaaaegaobaaaaaaaaaaaaceaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaadiaaaaahpcaabaaaaaaaaaaaegaobaaaacaaaaaa
egaobaaaaaaaaaaadiaaaaaihcaabaaaacaaaaaafgafbaaaaaaaaaaaegiccaaa
acaaaaaaahaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaacaaaaaaagaaaaaa
agaabaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
acaaaaaaaiaaaaaakgakbaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaaegiccaaaacaaaaaaajaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
aaaaaaahhccabaaaaeaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
"!!GLES


#ifdef VERTEX

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_9;
  highp vec3 tmpvar_10;
  tmpvar_9 = tmpvar_1.xyz;
  tmpvar_10 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_11;
  tmpvar_11[0].x = tmpvar_9.x;
  tmpvar_11[0].y = tmpvar_10.x;
  tmpvar_11[0].z = tmpvar_2.x;
  tmpvar_11[1].x = tmpvar_9.y;
  tmpvar_11[1].y = tmpvar_10.y;
  tmpvar_11[1].z = tmpvar_2.y;
  tmpvar_11[2].x = tmpvar_9.z;
  tmpvar_11[2].y = tmpvar_10.z;
  tmpvar_11[2].z = tmpvar_2.z;
  highp vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_11 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_12;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_14;
  tmpvar_14.w = 1.0;
  tmpvar_14.xyz = tmpvar_8;
  mediump vec3 tmpvar_15;
  mediump vec4 normal_16;
  normal_16 = tmpvar_14;
  highp float vC_17;
  mediump vec3 x3_18;
  mediump vec3 x2_19;
  mediump vec3 x1_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAr, normal_16);
  x1_20.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAg, normal_16);
  x1_20.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHAb, normal_16);
  x1_20.z = tmpvar_23;
  mediump vec4 tmpvar_24;
  tmpvar_24 = (normal_16.xyzz * normal_16.yzzx);
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBr, tmpvar_24);
  x2_19.x = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBg, tmpvar_24);
  x2_19.y = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = dot (unity_SHBb, tmpvar_24);
  x2_19.z = tmpvar_27;
  mediump float tmpvar_28;
  tmpvar_28 = ((normal_16.x * normal_16.x) - (normal_16.y * normal_16.y));
  vC_17 = tmpvar_28;
  highp vec3 tmpvar_29;
  tmpvar_29 = (unity_SHC.xyz * vC_17);
  x3_18 = tmpvar_29;
  tmpvar_15 = ((x1_20 + x2_19) + x3_18);
  shlight_3 = tmpvar_15;
  tmpvar_6 = shlight_3;
  highp vec3 tmpvar_30;
  tmpvar_30 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_31;
  tmpvar_31 = (unity_4LightPosX0 - tmpvar_30.x);
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosY0 - tmpvar_30.y);
  highp vec4 tmpvar_33;
  tmpvar_33 = (unity_4LightPosZ0 - tmpvar_30.z);
  highp vec4 tmpvar_34;
  tmpvar_34 = (((tmpvar_31 * tmpvar_31) + (tmpvar_32 * tmpvar_32)) + (tmpvar_33 * tmpvar_33));
  highp vec4 tmpvar_35;
  tmpvar_35 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_31 * tmpvar_8.x) + (tmpvar_32 * tmpvar_8.y)) + (tmpvar_33 * tmpvar_8.z)) * inversesqrt(tmpvar_34))) * (1.0/((1.0 + (tmpvar_34 * unity_4LightAtten0)))));
  highp vec3 tmpvar_36;
  tmpvar_36 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_35.x) + (unity_LightColor[1].xyz * tmpvar_35.y)) + (unity_LightColor[2].xyz * tmpvar_35.z)) + (unity_LightColor[3].xyz * tmpvar_35.w)));
  tmpvar_6 = tmpvar_36;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_5;
  tmpvar_5 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), tmpvar_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  highp vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_11;
  lightDir_11 = xlv_TEXCOORD2;
  mediump vec3 viewDir_12;
  viewDir_12 = tmpvar_10;
  mediump vec4 c_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (tmpvar_5, lightDir_11));
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (tmpvar_5, normalize((lightDir_11 + viewDir_12))));
  nh_14 = tmpvar_16;
  mediump float tmpvar_17;
  tmpvar_17 = ((1.0 - dot (normalize(viewDir_12), tmpvar_5)) + dot (normalize(lightDir_11), tmpvar_5));
  highp vec3 tmpvar_18;
  tmpvar_18 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_15) + ((((_LightColor0.xyz * pow (nh_14, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * 2.0);
  c_13.xyz = tmpvar_18;
  highp vec3 tmpvar_19;
  tmpvar_19 = (c_13.xyz + (c_13.xyz * (pow (tmpvar_17, _RimPower) * _RimColor.xyz)));
  c_13.xyz = tmpvar_19;
  c_13.w = tmpvar_9;
  c_1 = c_13;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
"!!GLES


#ifdef VERTEX

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_9;
  highp vec3 tmpvar_10;
  tmpvar_9 = tmpvar_1.xyz;
  tmpvar_10 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_11;
  tmpvar_11[0].x = tmpvar_9.x;
  tmpvar_11[0].y = tmpvar_10.x;
  tmpvar_11[0].z = tmpvar_2.x;
  tmpvar_11[1].x = tmpvar_9.y;
  tmpvar_11[1].y = tmpvar_10.y;
  tmpvar_11[1].z = tmpvar_2.y;
  tmpvar_11[2].x = tmpvar_9.z;
  tmpvar_11[2].y = tmpvar_10.z;
  tmpvar_11[2].z = tmpvar_2.z;
  highp vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_11 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_12;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_14;
  tmpvar_14.w = 1.0;
  tmpvar_14.xyz = tmpvar_8;
  mediump vec3 tmpvar_15;
  mediump vec4 normal_16;
  normal_16 = tmpvar_14;
  highp float vC_17;
  mediump vec3 x3_18;
  mediump vec3 x2_19;
  mediump vec3 x1_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAr, normal_16);
  x1_20.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAg, normal_16);
  x1_20.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHAb, normal_16);
  x1_20.z = tmpvar_23;
  mediump vec4 tmpvar_24;
  tmpvar_24 = (normal_16.xyzz * normal_16.yzzx);
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBr, tmpvar_24);
  x2_19.x = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBg, tmpvar_24);
  x2_19.y = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = dot (unity_SHBb, tmpvar_24);
  x2_19.z = tmpvar_27;
  mediump float tmpvar_28;
  tmpvar_28 = ((normal_16.x * normal_16.x) - (normal_16.y * normal_16.y));
  vC_17 = tmpvar_28;
  highp vec3 tmpvar_29;
  tmpvar_29 = (unity_SHC.xyz * vC_17);
  x3_18 = tmpvar_29;
  tmpvar_15 = ((x1_20 + x2_19) + x3_18);
  shlight_3 = tmpvar_15;
  tmpvar_6 = shlight_3;
  highp vec3 tmpvar_30;
  tmpvar_30 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_31;
  tmpvar_31 = (unity_4LightPosX0 - tmpvar_30.x);
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosY0 - tmpvar_30.y);
  highp vec4 tmpvar_33;
  tmpvar_33 = (unity_4LightPosZ0 - tmpvar_30.z);
  highp vec4 tmpvar_34;
  tmpvar_34 = (((tmpvar_31 * tmpvar_31) + (tmpvar_32 * tmpvar_32)) + (tmpvar_33 * tmpvar_33));
  highp vec4 tmpvar_35;
  tmpvar_35 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_31 * tmpvar_8.x) + (tmpvar_32 * tmpvar_8.y)) + (tmpvar_33 * tmpvar_8.z)) * inversesqrt(tmpvar_34))) * (1.0/((1.0 + (tmpvar_34 * unity_4LightAtten0)))));
  highp vec3 tmpvar_36;
  tmpvar_36 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_35.x) + (unity_LightColor[1].xyz * tmpvar_35.y)) + (unity_LightColor[2].xyz * tmpvar_35.z)) + (unity_LightColor[3].xyz * tmpvar_35.w)));
  tmpvar_6 = tmpvar_36;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 normal_5;
  normal_5.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_5.z = sqrt((1.0 - clamp (dot (normal_5.xy, normal_5.xy), 0.0, 1.0)));
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), normal_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  highp vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_11;
  lightDir_11 = xlv_TEXCOORD2;
  mediump vec3 viewDir_12;
  viewDir_12 = tmpvar_10;
  mediump vec4 c_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (normal_5, lightDir_11));
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (normal_5, normalize((lightDir_11 + viewDir_12))));
  nh_14 = tmpvar_16;
  mediump float tmpvar_17;
  tmpvar_17 = ((1.0 - dot (normalize(viewDir_12), normal_5)) + dot (normalize(lightDir_11), normal_5));
  highp vec3 tmpvar_18;
  tmpvar_18 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_15) + ((((_LightColor0.xyz * pow (nh_14, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * 2.0);
  c_13.xyz = tmpvar_18;
  highp vec3 tmpvar_19;
  tmpvar_19 = (c_13.xyz + (c_13.xyz * (pow (tmpvar_17, _RimPower) * _RimColor.xyz)));
  c_13.xyz = tmpvar_19;
  c_13.w = tmpvar_9;
  c_1 = c_13;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_WorldSpaceLightPos0]
Vector 14 [unity_4LightPosX0]
Vector 15 [unity_4LightPosY0]
Vector 16 [unity_4LightPosZ0]
Vector 17 [unity_4LightAtten0]
Vector 18 [unity_LightColor0]
Vector 19 [unity_LightColor1]
Vector 20 [unity_LightColor2]
Vector 21 [unity_LightColor3]
Vector 22 [unity_SHAr]
Vector 23 [unity_SHAg]
Vector 24 [unity_SHAb]
Vector 25 [unity_SHBr]
Vector 26 [unity_SHBg]
Vector 27 [unity_SHBb]
Vector 28 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 29 [unity_Scale]
Vector 30 [_MainTex_ST]
Vector 31 [_BumpMap_ST]
"agal_vs
c32 1.0 0.0 0.0 0.0
[bc]
adaaaaaaadaaahacabaaaaoeaaaaaaaabnaaaappabaaaaaa mul r3.xyz, a1, c29.w
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.x, a0, c5
bfaaaaaaabaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.x, r0.x
abaaaaaaabaaapacabaaaaaaacaaaaaaapaaaaoeabaaaaaa add r1, r1.x, c15
bcaaaaaaadaaaiacadaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r3.w, r3.xyzz, c5
bcaaaaaaaeaaabacadaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r4.x, r3.xyzz, c4
bcaaaaaaadaaabacadaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r3.x, r3.xyzz, c6
adaaaaaaacaaapacadaaaappacaaaaaaabaaaaoeacaaaaaa mul r2, r3.w, r1
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaapacaaaaaaaaacaaaaaaaoaaaaoeabaaaaaa add r0, r0.x, c14
adaaaaaaabaaapacabaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r1, r1, r1
aaaaaaaaaeaaaeacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r4.z, r3.x
adaaaaaaafaaapacaeaaaaaaacaaaaaaaaaaaaoeacaaaaaa mul r5, r4.x, r0
abaaaaaaacaaapacafaaaaoeacaaaaaaacaaaaoeacaaaaaa add r2, r5, r2
aaaaaaaaaeaaaiaccaaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r4.w, c32.x
bdaaaaaaaeaaacacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r4.y, a0, c6
adaaaaaaafaaapacaaaaaaoeacaaaaaaaaaaaaoeacaaaaaa mul r5, r0, r0
abaaaaaaabaaapacafaaaaoeacaaaaaaabaaaaoeacaaaaaa add r1, r5, r1
bfaaaaaaaaaaacacaeaaaaffacaaaaaaaaaaaaaaaaaaaaaa neg r0.y, r4.y
abaaaaaaaaaaapacaaaaaaffacaaaaaabaaaaaoeabaaaaaa add r0, r0.y, c16
adaaaaaaafaaapacaaaaaaoeacaaaaaaaaaaaaoeacaaaaaa mul r5, r0, r0
abaaaaaaabaaapacafaaaaoeacaaaaaaabaaaaoeacaaaaaa add r1, r5, r1
adaaaaaaaaaaapacadaaaaaaacaaaaaaaaaaaaoeacaaaaaa mul r0, r3.x, r0
abaaaaaaaaaaapacaaaaaaoeacaaaaaaacaaaaoeacaaaaaa add r0, r0, r2
adaaaaaaacaaapacabaaaaoeacaaaaaabbaaaaoeabaaaaaa mul r2, r1, c17
aaaaaaaaaeaaacacadaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r4.y, r3.w
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
akaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rsq r1.y, r1.y
akaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rsq r1.w, r1.w
akaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rsq r1.z, r1.z
adaaaaaaaaaaapacaaaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r0, r0, r1
abaaaaaaabaaapacacaaaaoeacaaaaaacaaaaaaaabaaaaaa add r1, r2, c32.x
bdaaaaaaacaaaeacaeaaaaoeacaaaaaabiaaaaoeabaaaaaa dp4 r2.z, r4, c24
bdaaaaaaacaaacacaeaaaaoeacaaaaaabhaaaaoeabaaaaaa dp4 r2.y, r4, c23
bdaaaaaaacaaabacaeaaaaoeacaaaaaabgaaaaoeabaaaaaa dp4 r2.x, r4, c22
afaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r1.x, r1.x
afaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rcp r1.y, r1.y
afaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rcp r1.w, r1.w
afaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rcp r1.z, r1.z
ahaaaaaaaaaaapacaaaaaaoeacaaaaaacaaaaaffabaaaaaa max r0, r0, c32.y
adaaaaaaaaaaapacaaaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r0, r0, r1
adaaaaaaabaaahacaaaaaaffacaaaaaabdaaaaoeabaaaaaa mul r1.xyz, r0.y, c19
adaaaaaaafaaahacaaaaaaaaacaaaaaabcaaaaoeabaaaaaa mul r5.xyz, r0.x, c18
abaaaaaaabaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r5.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakkacaaaaaabeaaaaoeabaaaaaa mul r0.xyz, r0.z, c20
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaabaaahacaaaaaappacaaaaaabfaaaaoeabaaaaaa mul r1.xyz, r0.w, c21
abaaaaaaabaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa add r1.xyz, r1.xyzz, r0.xyzz
adaaaaaaaaaaapacaeaaaakeacaaaaaaaeaaaacjacaaaaaa mul r0, r4.xyzz, r4.yzzx
adaaaaaaabaaaiacadaaaappacaaaaaaadaaaappacaaaaaa mul r1.w, r3.w, r3.w
bdaaaaaaadaaaeacaaaaaaoeacaaaaaablaaaaoeabaaaaaa dp4 r3.z, r0, c27
bdaaaaaaadaaacacaaaaaaoeacaaaaaabkaaaaoeabaaaaaa dp4 r3.y, r0, c26
bdaaaaaaadaaabacaaaaaaoeacaaaaaabjaaaaoeabaaaaaa dp4 r3.x, r0, c25
adaaaaaaafaaaiacaeaaaaaaacaaaaaaaeaaaaaaacaaaaaa mul r5.w, r4.x, r4.x
acaaaaaaabaaaiacafaaaappacaaaaaaabaaaappacaaaaaa sub r1.w, r5.w, r1.w
adaaaaaaaaaaahacabaaaappacaaaaaabmaaaaoeabaaaaaa mul r0.xyz, r1.w, c28
abaaaaaaacaaahacacaaaakeacaaaaaaadaaaakeacaaaaaa add r2.xyz, r2.xyzz, r3.xyzz
abaaaaaaacaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa add r2.xyz, r2.xyzz, r0.xyzz
abaaaaaaadaaahaeacaaaakeacaaaaaaabaaaakeacaaaaaa add v3.xyz, r2.xyzz, r1.xyzz
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaafaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r5.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r5.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c13, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
aaaaaaaaabaaaiaccaaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c32.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaeaaacacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c13, r0
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaafaaahacacaaaakeacaaaaaabnaaaappabaaaaaa mul r5.xyz, r2.xyzz, c29.w
acaaaaaaacaaahacafaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r5.xyzz, a0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacanaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c13, r1
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
adaaaaaaafaaamacadaaaaeeaaaaaaaabpaaaaeeabaaaaaa mul r5.zw, a3.xyxy, c31.xyxy
abaaaaaaaaaaamaeafaaaaopacaaaaaabpaaaaoeabaaaaaa add v0.zw, r5.wwzw, c31
adaaaaaaafaaadacadaaaaoeaaaaaaaaboaaaaoeabaaaaaa mul r5.xy, a3, c30
abaaaaaaaaaaadaeafaaaafeacaaaaaaboaaaaooabaaaaaa add v0.xy, r5.xyyy, c30.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 128 used size, 8 vars
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 32 [unity_4LightPosX0] 4
Vector 48 [unity_4LightPosY0] 4
Vector 64 [unity_4LightPosZ0] 4
Vector 80 [unity_4LightAtten0] 4
Vector 96 [unity_LightColor0] 4
Vector 112 [unity_LightColor1] 4
Vector 128 [unity_LightColor2] 4
Vector 144 [unity_LightColor3] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 65 instructions, 6 temp regs, 0 temp arrays:
// ALU 36 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecededekhjfkopoilglnkieacnhkolkjgmngabaaaaaapabaaaaaaeaaaaaa
daaaaaaapmafaaaaiiapaaaafabaaaaaebgpgodjmeafaaaameafaaaaaaacpopp
eiafaaaahmaaaaaaahaaceaaaaaahiaaaaaahiaaaaaaceaaabaahiaaaaaaagaa
acaaabaaaaaaaaaaabaaaeaaabaaadaaaaaaaaaaacaaaaaaabaaaeaaaaaaaaaa
acaaacaaaiaaafaaaaaaaaaaacaabcaaahaaanaaaaaaaaaaadaaaaaaaeaabeaa
aaaaaaaaadaaamaaajaabiaaaaaaaaaaaaaaaaaaaaacpoppfbaaaaafcbaaapka
aaaaiadpaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaac
afaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapja
aeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaaeaaaaaeaaaaamoaadaaeeja
acaaeekaacaaoekaabaaaaacaaaaapiaaeaaoekaafaaaaadabaaahiaaaaaffia
bnaaoekaaeaaaaaeabaaahiabmaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahia
boaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiabpaaoekaaaaappiaaaaaoeia
aiaaaaadacaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaacaaoejaafaaaaad
acaaahiaabaanciaabaamjjaaeaaaaaeabaaahiaabaamjiaabaancjaacaaoeib
afaaaaadabaaahiaabaaoeiaabaappjaaiaaaaadacaaacoaabaaoeiaaaaaoeia
aiaaaaadacaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaadaaoekaafaaaaad
acaaahiaaaaaffiabnaaoekaaeaaaaaeaaaaaliabmaakekaaaaaaaiaacaakeia
aeaaaaaeaaaaahiaboaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeia
bpaaoekaaeaaaaaeaaaaahiaaaaaoeiacaaappkaaaaaoejbaiaaaaadabaaaboa
abaaoejaaaaaoeiaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoa
acaaoejaaaaaoeiaafaaaaadaaaaahiaaaaaffjabjaaoekaaeaaaaaeaaaaahia
biaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaahiabkaaoekaaaaakkjaaaaaoeia
aeaaaaaeaaaaahiablaaoekaaaaappjaaaaaoeiaacaaaaadabaaapiaaaaakkib
ahaaoekaacaaaaadacaaapiaaaaaaaibafaaoekaacaaaaadaaaaapiaaaaaffib
agaaoekaafaaaaadadaaahiaacaaoejacaaappkaafaaaaadaeaaahiaadaaffia
bjaaoekaaeaaaaaeadaaaliabiaakekaadaaaaiaaeaakeiaaeaaaaaeadaaahia
bkaaoekaadaakkiaadaapeiaafaaaaadaeaaapiaaaaaoeiaadaaffiaafaaaaad
aaaaapiaaaaaoeiaaaaaoeiaaeaaaaaeaaaaapiaacaaoeiaacaaoeiaaaaaoeia
aeaaaaaeacaaapiaacaaoeiaadaaaaiaaeaaoeiaaeaaaaaeacaaapiaabaaoeia
adaakkiaacaaoeiaaeaaaaaeaaaaapiaabaaoeiaabaaoeiaaaaaoeiaahaaaaac
abaaabiaaaaaaaiaahaaaaacabaaaciaaaaaffiaahaaaaacabaaaeiaaaaakkia
ahaaaaacabaaaiiaaaaappiaabaaaaacaeaaabiacbaaaakaaeaaaaaeaaaaapia
aaaaoeiaaiaaoekaaeaaaaiaafaaaaadabaaapiaabaaoeiaacaaoeiaalaaaaad
abaaapiaabaaoeiacbaaffkaagaaaaacacaaabiaaaaaaaiaagaaaaacacaaacia
aaaaffiaagaaaaacacaaaeiaaaaakkiaagaaaaacacaaaiiaaaaappiaafaaaaad
aaaaapiaabaaoeiaacaaoeiaafaaaaadabaaahiaaaaaffiaakaaoekaaeaaaaae
abaaahiaajaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiaalaaoekaaaaakkia
abaaoeiaaeaaaaaeaaaaahiaamaaoekaaaaappiaaaaaoeiaabaaaaacadaaaiia
cbaaaakaajaaaaadabaaabiaanaaoekaadaaoeiaajaaaaadabaaaciaaoaaoeka
adaaoeiaajaaaaadabaaaeiaapaaoekaadaaoeiaafaaaaadacaaapiaadaacjia
adaakeiaajaaaaadaeaaabiabaaaoekaacaaoeiaajaaaaadaeaaaciabbaaoeka
acaaoeiaajaaaaadaeaaaeiabcaaoekaacaaoeiaacaaaaadabaaahiaabaaoeia
aeaaoeiaafaaaaadaaaaaiiaadaaffiaadaaffiaaeaaaaaeaaaaaiiaadaaaaia
adaaaaiaaaaappibaeaaaaaeabaaahiabdaaoekaaaaappiaabaaoeiaacaaaaad
adaaahoaaaaaoeiaabaaoeiaafaaaaadaaaaapiaaaaaffjabfaaoekaaeaaaaae
aaaaapiabeaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiabgaaoekaaaaakkja
aaaaoeiaaeaaaaaeaaaaapiabhaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadma
aaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaafdeieefc
ieajaaaaeaaaabaagbacaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaac
agaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
abaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
acaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaa
egiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaal
dccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaa
aaaaaaaaagaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaa
aaaaaaaaahaaaaaakgiocaaaaaaaaaaaahaaaaaadiaaaaahhcaabaaaaaaaaaaa
jgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaa
acaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaa
acaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaa
adaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaa
egbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaa
acaaaaaaegacbaaaabaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadp
diaaaaaihcaabaaaabaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaa
diaaaaaihcaabaaaacaaaaaafgafbaaaabaaaaaaegiccaaaadaaaaaaanaaaaaa
dcaaaaaklcaabaaaabaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaabaaaaaa
egaibaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaa
kgakbaaaabaaaaaaegadbaaaabaaaaaabbaaaaaibcaabaaaabaaaaaaegiocaaa
acaaaaaabcaaaaaaegaobaaaaaaaaaaabbaaaaaiccaabaaaabaaaaaaegiocaaa
acaaaaaabdaaaaaaegaobaaaaaaaaaaabbaaaaaiecaabaaaabaaaaaaegiocaaa
acaaaaaabeaaaaaaegaobaaaaaaaaaaadiaaaaahpcaabaaaacaaaaaajgacbaaa
aaaaaaaaegakbaaaaaaaaaaabbaaaaaibcaabaaaadaaaaaaegiocaaaacaaaaaa
bfaaaaaaegaobaaaacaaaaaabbaaaaaiccaabaaaadaaaaaaegiocaaaacaaaaaa
bgaaaaaaegaobaaaacaaaaaabbaaaaaiecaabaaaadaaaaaaegiocaaaacaaaaaa
bhaaaaaaegaobaaaacaaaaaaaaaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
egacbaaaadaaaaaadiaaaaahicaabaaaaaaaaaaabkaabaaaaaaaaaaabkaabaaa
aaaaaaaadcaaaaakicaabaaaaaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaa
dkaabaiaebaaaaaaaaaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaacaaaaaa
biaaaaaapgapbaaaaaaaaaaaegacbaaaabaaaaaadiaaaaaihcaabaaaacaaaaaa
fgbfbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaacaaaaaa
egiccaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaaacaaaaaadcaaaaak
hcaabaaaacaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaa
acaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaa
aaaaaaaaegacbaaaacaaaaaaaaaaaaajpcaabaaaadaaaaaafgafbaiaebaaaaaa
acaaaaaaegiocaaaacaaaaaaadaaaaaadiaaaaahpcaabaaaaeaaaaaafgafbaaa
aaaaaaaaegaobaaaadaaaaaadiaaaaahpcaabaaaadaaaaaaegaobaaaadaaaaaa
egaobaaaadaaaaaaaaaaaaajpcaabaaaafaaaaaaagaabaiaebaaaaaaacaaaaaa
egiocaaaacaaaaaaacaaaaaaaaaaaaajpcaabaaaacaaaaaakgakbaiaebaaaaaa
acaaaaaaegiocaaaacaaaaaaaeaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaa
afaaaaaaagaabaaaaaaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaaaaaaaaaa
egaobaaaacaaaaaakgakbaaaaaaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaa
adaaaaaaegaobaaaafaaaaaaegaobaaaafaaaaaaegaobaaaadaaaaaadcaaaaaj
pcaabaaaacaaaaaaegaobaaaacaaaaaaegaobaaaacaaaaaaegaobaaaadaaaaaa
eeaaaaafpcaabaaaadaaaaaaegaobaaaacaaaaaadcaaaaanpcaabaaaacaaaaaa
egaobaaaacaaaaaaegiocaaaacaaaaaaafaaaaaaaceaaaaaaaaaiadpaaaaiadp
aaaaiadpaaaaiadpaoaaaaakpcaabaaaacaaaaaaaceaaaaaaaaaiadpaaaaiadp
aaaaiadpaaaaiadpegaobaaaacaaaaaadiaaaaahpcaabaaaaaaaaaaaegaobaaa
aaaaaaaaegaobaaaadaaaaaadeaaaaakpcaabaaaaaaaaaaaegaobaaaaaaaaaaa
aceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadiaaaaahpcaabaaaaaaaaaaa
egaobaaaacaaaaaaegaobaaaaaaaaaaadiaaaaaihcaabaaaacaaaaaafgafbaaa
aaaaaaaaegiccaaaacaaaaaaahaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaa
acaaaaaaagaaaaaaagaabaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaaegiccaaaacaaaaaaaiaaaaaakgakbaaaaaaaaaaaegacbaaaacaaaaaa
dcaaaaakhcaabaaaaaaaaaaaegiccaaaacaaaaaaajaaaaaapgapbaaaaaaaaaaa
egacbaaaaaaaaaaaaaaaaaahhccabaaaaeaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
laaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
afaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaa
feeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaaaiaaaaaa
iaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahaiaaaa
imaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfaepfdejfe
ejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 400
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 415
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 384
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 407
#line 424
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 95
highp vec3 Shade4PointLights( in highp vec4 lightPosX, in highp vec4 lightPosY, in highp vec4 lightPosZ, in highp vec3 lightColor0, in highp vec3 lightColor1, in highp vec3 lightColor2, in highp vec3 lightColor3, in highp vec4 lightAttenSq, in highp vec3 pos, in highp vec3 normal ) {
    highp vec4 toLightX = (lightPosX - pos.x);
    highp vec4 toLightY = (lightPosY - pos.y);
    #line 99
    highp vec4 toLightZ = (lightPosZ - pos.z);
    highp vec4 lengthSq = vec4( 0.0);
    lengthSq += (toLightX * toLightX);
    lengthSq += (toLightY * toLightY);
    #line 103
    lengthSq += (toLightZ * toLightZ);
    highp vec4 ndotl = vec4( 0.0);
    ndotl += (toLightX * normal.x);
    ndotl += (toLightY * normal.y);
    #line 107
    ndotl += (toLightZ * normal.z);
    highp vec4 corr = inversesqrt(lengthSq);
    ndotl = max( vec4( 0.0, 0.0, 0.0, 0.0), (ndotl * corr));
    highp vec4 atten = (1.0 / (1.0 + (lengthSq * lightAttenSq)));
    #line 111
    highp vec4 diff = (ndotl * atten);
    highp vec3 col = vec3( 0.0);
    col += (lightColor0 * diff.x);
    col += (lightColor1 * diff.y);
    #line 115
    col += (lightColor2 * diff.z);
    col += (lightColor3 * diff.w);
    return col;
}
#line 136
mediump vec3 ShadeSH9( in mediump vec4 normal ) {
    mediump vec3 x1;
    mediump vec3 x2;
    mediump vec3 x3;
    x1.x = dot( unity_SHAr, normal);
    #line 140
    x1.y = dot( unity_SHAg, normal);
    x1.z = dot( unity_SHAb, normal);
    mediump vec4 vB = (normal.xyzz * normal.yzzx);
    x2.x = dot( unity_SHBr, vB);
    #line 144
    x2.y = dot( unity_SHBg, vB);
    x2.z = dot( unity_SHBb, vB);
    highp float vC = ((normal.x * normal.x) - (normal.y * normal.y));
    x3 = (unity_SHC.xyz * vC);
    #line 148
    return ((x1 + x2) + x3);
}
#line 426
v2f_surf vert_surf( in appdata_full v ) {
    #line 428
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 432
    highp vec3 worldN = (mat3( _Object2World) * (v.normal * unity_Scale.w));
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    #line 436
    o.lightDir = lightDir;
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    highp vec3 shlight = ShadeSH9( vec4( worldN, 1.0));
    #line 440
    o.vlight = shlight;
    highp vec3 worldPos = (_Object2World * v.vertex).xyz;
    o.vlight += Shade4PointLights( unity_4LightPosX0, unity_4LightPosY0, unity_4LightPosZ0, unity_LightColor[0].xyz, unity_LightColor[1].xyz, unity_LightColor[2].xyz, unity_LightColor[3].xyz, unity_4LightAtten0, worldPos, worldN);
    #line 444
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out highp vec3 xlv_TEXCOORD1;
out lowp vec3 xlv_TEXCOORD2;
out lowp vec3 xlv_TEXCOORD3;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD2 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD3 = vec3(xl_retval.vlight);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 400
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 415
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 384
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 407
#line 424
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 387
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    #line 389
    mediump vec3 h = normalize((lightDir + viewDir));
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    #line 393
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    #line 397
    c.w = s.Alpha;
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 407
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 411
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 446
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 448
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    surfIN.viewDir = IN.viewDir;
    #line 452
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    #line 456
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp float atten = 1.0;
    #line 460
    lowp vec4 c = vec4( 0.0);
    c = LightingSimpleSpecular( o, IN.lightDir, normalize(IN.viewDir), atten);
    c.xyz += (o.Albedo * IN.vlight);
    c.xyz += o.Emission;
    #line 464
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in highp vec3 xlv_TEXCOORD1;
in lowp vec3 xlv_TEXCOORD2;
in lowp vec3 xlv_TEXCOORD3;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD1);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD2);
    xlt_IN.vlight = vec3(xlv_TEXCOORD3);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_ProjectionParams]
Vector 15 [_WorldSpaceLightPos0]
Vector 16 [unity_4LightPosX0]
Vector 17 [unity_4LightPosY0]
Vector 18 [unity_4LightPosZ0]
Vector 19 [unity_4LightAtten0]
Vector 20 [unity_LightColor0]
Vector 21 [unity_LightColor1]
Vector 22 [unity_LightColor2]
Vector 23 [unity_LightColor3]
Vector 24 [unity_SHAr]
Vector 25 [unity_SHAg]
Vector 26 [unity_SHAb]
Vector 27 [unity_SHBr]
Vector 28 [unity_SHBg]
Vector 29 [unity_SHBb]
Vector 30 [unity_SHC]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 31 [unity_Scale]
Vector 32 [_MainTex_ST]
Vector 33 [_BumpMap_ST]
"!!ARBvp1.0
# 80 ALU
PARAM c[34] = { { 1, 0, 0.5 },
		state.matrix.mvp,
		program.local[5..33] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
MUL R3.xyz, vertex.normal, c[31].w;
DP4 R0.x, vertex.position, c[6];
ADD R1, -R0.x, c[17];
DP3 R3.w, R3, c[6];
DP3 R4.x, R3, c[5];
DP3 R3.x, R3, c[7];
MUL R2, R3.w, R1;
DP4 R0.x, vertex.position, c[5];
ADD R0, -R0.x, c[16];
MUL R1, R1, R1;
MOV R4.z, R3.x;
MAD R2, R4.x, R0, R2;
MOV R4.w, c[0].x;
DP4 R4.y, vertex.position, c[7];
MAD R1, R0, R0, R1;
ADD R0, -R4.y, c[18];
MAD R1, R0, R0, R1;
MAD R0, R3.x, R0, R2;
MUL R2, R1, c[19];
MOV R4.y, R3.w;
RSQ R1.x, R1.x;
RSQ R1.y, R1.y;
RSQ R1.w, R1.w;
RSQ R1.z, R1.z;
MUL R0, R0, R1;
ADD R1, R2, c[0].x;
RCP R1.x, R1.x;
RCP R1.y, R1.y;
RCP R1.w, R1.w;
RCP R1.z, R1.z;
MAX R0, R0, c[0].y;
MUL R0, R0, R1;
MUL R1.xyz, R0.y, c[21];
MAD R1.xyz, R0.x, c[20], R1;
MAD R0.xyz, R0.z, c[22], R1;
MAD R1.xyz, R0.w, c[23], R0;
MUL R0, R4.xyzz, R4.yzzx;
DP4 R3.z, R0, c[29];
DP4 R3.y, R0, c[28];
DP4 R3.x, R0, c[27];
MUL R1.w, R3, R3;
MOV R0.w, c[0].x;
MAD R0.x, R4, R4, -R1.w;
DP4 R2.z, R4, c[26];
DP4 R2.y, R4, c[25];
DP4 R2.x, R4, c[24];
ADD R2.xyz, R2, R3;
MUL R3.xyz, R0.x, c[30];
ADD R3.xyz, R2, R3;
MOV R0.xyz, vertex.attrib[14];
MUL R2.xyz, vertex.normal.zxyw, R0.yzxw;
ADD result.texcoord[3].xyz, R3, R1;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R2;
MOV R0.xyz, c[13];
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[31].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[15];
DP4 R3.z, R1, c[11];
DP4 R3.x, R1, c[9];
DP4 R3.y, R1, c[10];
DP3 result.texcoord[1].y, R0, R2;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.w, vertex.position, c[4];
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].z;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
ADD result.texcoord[4].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[4].zw, R0;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[33].xyxy, c[33];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[32], c[32].zwzw;
END
# 80 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_ScreenParams]
Vector 15 [_WorldSpaceLightPos0]
Vector 16 [unity_4LightPosX0]
Vector 17 [unity_4LightPosY0]
Vector 18 [unity_4LightPosZ0]
Vector 19 [unity_4LightAtten0]
Vector 20 [unity_LightColor0]
Vector 21 [unity_LightColor1]
Vector 22 [unity_LightColor2]
Vector 23 [unity_LightColor3]
Vector 24 [unity_SHAr]
Vector 25 [unity_SHAg]
Vector 26 [unity_SHAb]
Vector 27 [unity_SHBr]
Vector 28 [unity_SHBg]
Vector 29 [unity_SHBb]
Vector 30 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 31 [unity_Scale]
Vector 32 [_MainTex_ST]
Vector 33 [_BumpMap_ST]
"vs_2_0
; 83 ALU
def c34, 1.00000000, 0.00000000, 0.50000000, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r3.xyz, v2, c31.w
dp4 r0.x, v0, c5
add r1, -r0.x, c17
dp3 r3.w, r3, c5
dp3 r4.x, r3, c4
dp3 r3.x, r3, c6
mul r2, r3.w, r1
dp4 r0.x, v0, c4
add r0, -r0.x, c16
mul r1, r1, r1
mov r4.z, r3.x
mad r2, r4.x, r0, r2
mov r4.w, c34.x
dp4 r4.y, v0, c6
mad r1, r0, r0, r1
add r0, -r4.y, c18
mad r1, r0, r0, r1
mad r0, r3.x, r0, r2
mul r2, r1, c19
mov r4.y, r3.w
rsq r1.x, r1.x
rsq r1.y, r1.y
rsq r1.w, r1.w
rsq r1.z, r1.z
mul r0, r0, r1
add r1, r2, c34.x
dp4 r2.z, r4, c26
dp4 r2.y, r4, c25
dp4 r2.x, r4, c24
rcp r1.x, r1.x
rcp r1.y, r1.y
rcp r1.w, r1.w
rcp r1.z, r1.z
max r0, r0, c34.y
mul r0, r0, r1
mul r1.xyz, r0.y, c21
mad r1.xyz, r0.x, c20, r1
mad r0.xyz, r0.z, c22, r1
mad r1.xyz, r0.w, c23, r0
mul r0, r4.xyzz, r4.yzzx
mul r1.w, r3, r3
dp4 r3.z, r0, c29
dp4 r3.y, r0, c28
dp4 r3.x, r0, c27
mad r1.w, r4.x, r4.x, -r1
mul r0.xyz, r1.w, c30
add r2.xyz, r2, r3
add r2.xyz, r2, r0
add oT3.xyz, r2, r1
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c15, r0
mov r0, c9
dp4 r4.y, c15, r0
mov r1.w, c34.x
mov r1.xyz, c12
dp4 r0.w, v0, c3
dp4 r0.z, v0, c2
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c31.w, -v0
mov r1, c8
dp4 r4.x, c15, r1
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c34.z
mul r1.y, r1, c13.x
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
mad oT4.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT4.zw, r0
mad oT0.zw, v3.xyxy, c33.xyxy, c33
mad oT0.xy, v3, c32, c32.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 33 [_BumpMap_ST]
Vector 32 [_MainTex_ST]
Matrix 23 [_Object2World] 4
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 27 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 3 [_WorldSpaceLightPos0]
Matrix 19 [glstate_matrix_mvp] 4
Vector 7 [unity_4LightAtten0]
Vector 4 [unity_4LightPosX0]
Vector 5 [unity_4LightPosY0]
Vector 6 [unity_4LightPosZ0]
Vector 8 [unity_LightColor0]
Vector 9 [unity_LightColor1]
Vector 10 [unity_LightColor2]
Vector 11 [unity_LightColor3]
Vector 14 [unity_SHAb]
Vector 13 [unity_SHAg]
Vector 12 [unity_SHAr]
Vector 17 [unity_SHBb]
Vector 16 [unity_SHBg]
Vector 15 [unity_SHBr]
Vector 18 [unity_SHC]
Vector 31 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 96.00 (72 instructions), vertex: 32, texture: 0,
//   sequencer: 32,  13 GPRs, 12 threads,
// Performance (if enough threads): ~96 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaaediaaaaaedmaaaaaaaaaaaaaaceaaaaadjiaaaaadmaaaaaaaaa
aaaaaaaaaaaaadhaaaaaaabmaaaaadgdpppoadaaaaaaaabgaaaaaabmaaaaaaaa
aaaaadfmaaaaabneaaacaacbaaabaaaaaaaaaboaaaaaaaaaaaaaabpaaaacaaca
aaabaaaaaaaaaboaaaaaaaaaaaaaabpmaaacaabhaaaeaaaaaaaaacamaaaaaaaa
aaaaacbmaaacaaabaaabaaaaaaaaaboaaaaaaaaaaaaaaccoaaacaaacaaabaaaa
aaaaaboaaaaaaaaaaaaaacdmaaacaablaaaeaaaaaaaaacamaaaaaaaaaaaaacek
aaacaaaaaaabaaaaaaaaacgaaaaaaaaaaaaaachaaaacaaadaaabaaaaaaaaaboa
aaaaaaaaaaaaacifaaacaabdaaaeaaaaaaaaacamaaaaaaaaaaaaacjiaaacaaah
aaabaaaaaaaaaboaaaaaaaaaaaaaacklaaacaaaeaaabaaaaaaaaaboaaaaaaaaa
aaaaaclnaaacaaafaaabaaaaaaaaaboaaaaaaaaaaaaaacmpaaacaaagaaabaaaa
aaaaaboaaaaaaaaaaaaaacobaaacaaaiaaaeaaaaaaaaacpeaaaaaaaaaaaaadae
aaacaaaoaaabaaaaaaaaaboaaaaaaaaaaaaaadapaaacaaanaaabaaaaaaaaaboa
aaaaaaaaaaaaadbkaaacaaamaaabaaaaaaaaaboaaaaaaaaaaaaaadcfaaacaabb
aaabaaaaaaaaaboaaaaaaaaaaaaaaddaaaacaabaaaabaaaaaaaaaboaaaaaaaaa
aaaaaddlaaacaaapaaabaaaaaaaaaboaaaaaaaaaaaaaadegaaacaabcaaabaaaa
aaaaaboaaaaaaaaaaaaaadfaaaacaabpaaabaaaaaaaaaboaaaaaaaaafpechfgn
haengbhafpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhi
fpfdfeaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaa
aaaaaaaafpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhc
gbgnhdaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgn
gfhcgbfagphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdha
gbgdgfemgjghgihefagphddaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaa
hfgogjhehjfpdeemgjghgiheebhehegfgodaaahfgogjhehjfpdeemgjghgihefa
gphdfidaaahfgogjhehjfpdeemgjghgihefagphdfjdaaahfgogjhehjfpdeemgj
ghgihefagphdfkdaaahfgogjhehjfpemgjghgiheedgpgmgphcaaklklaaabaaad
aaabaaaeaaaeaaaaaaaaaaaahfgogjhehjfpfdeiebgcaahfgogjhehjfpfdeieb
ghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdeiecgcaahfgogjhehjfpfdei
ecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfdeiedaahfgogjhehjfpfdgd
gbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaab
aaaaaaaaaaaaaaaaaaaaaabeaapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaeaaaaaadpmaaebaaamaaaaaaaaaaaaaaaaaaaaeekfaaaaaaabaaaaaaae
aaaaaaalaaaaacjaaabaaaaiaaaagaajaaaadaakaacafaalaaaapafaaaachbfb
aaafhcfcaaaihdfdaaajpefeaaaaaacpaaaabadaaaaaaacjaaaaaackaaaabacl
aaaaaacmaaaaaacnaaaabacoaaaabafdaaaaaaciaaaabadlaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaadpaaaaaaaaaaaaaadpiaaaaaaaaaaaaapaffeaaiaaaabcaa
mcaaaaaaaaaafaamaaaabcaameaaaaaaaaaagabbgabhbcaabcaaaaaaaaaagabn
gacdbcaabcaaaaaaaaaagacjgacpbcaabcaaaaaaaaaagadfgadlbcaabcaaaaaa
aaaagaebgaehbcaabcaaaaaaaaaagaenbafdbcaaccaaaaaaafpibaaaaaaaaanb
aaaaaaaaafpikaaaaaaaagiiaaaaaaaaafpieaaaaaaaaoiiaaaaaaaaafpidaaa
aaaaacdpaaaaaaaamiapaaaaaamgiiaakbabbgaamiapaaaaaalbnapiklabbfaa
miapaaaaaagmdepiklabbeaamiapaaamaablnajeklabbdaamiapiadoaananaaa
ocamamaamiahaaaaaamamgmaalbnaabomiahaaacaaleblaacbboadaamiahaaai
aamamgleclbnadacmiahaaahaalogfaaobaeakaamiahaaagaalelbleclbmaaaa
mialaaaaaagfblaakbaebpaamiahaaacaamgleaakbabbkaamiahaaacaalbmale
klabbjacmiahaaafaalbleaakbaabjaamiahaaagaamagmleclblaaagmiahaaah
abgflomaolaeakahmiahaaaiaalelbleclbmadaimiahaaaiaamagmleclbladai
miahaaajaamablaaobahakaamiahaaalabmabllpklagbpabmiahaaaaaagmlema
klaabiafmiahaaabaagmleleklabbiacmialaaabaabllemaklabbhabmiahaaah
aabllemaklaabhaaceihahaaaamagmgmkbamppiaaibpadafaalehcgmobahahah
aicpadagaegmaamgkaabaeahbeapaaacaflbaablkaabagabmiamiaaeaanlnlaa
ocamamaamiabiaabaaloloaapaalakaamiaciaabaaloloaapaajalaamiaeiaab
aaloloaapaalaeaamiabiaacaaloloaapaaiakaamiaciaacaaloloaapaajaiaa
miaeiaacaaloloaapaaiaeaamiadiaaaaabklabkiladcacamiamiaaaaaagkmag
iladcbcbaebbaiaeaadoangmepamahafbeacaaaeabdoanblgpanahabaeceaiae
aadoanlbepaoahafbeabaaababkhkhblkpafapabaeecaiabaakhkhmgipafbaaf
beaeaaababkhkhblkpafbbabaeipaiafaapipiblmbacacafkiipaaacaapilbeb
mbacahabmiapaaafaajejepiolaiaiafmiapaaacaajemgpiolaiahacmiadiaae
aamgbkbiklaaacaamiapaaacaajegmaaolagahacmiapaaaaaaaaaapiolagagaf
geihababaalologboaaeabadmiahaaabaabllemnklabbcabmiapaaaeaapipimg
ilaaahppfibaaaaaaaaaaagmocaaaaiaficaaaaaaaaaaalbocaaaaiafieaaaaa
aaaaaamgocaaaaiafiiaaaaaaaaaaablocaaaaiamiapaaaaaapiaaaaobacaaaa
emipaaadaapilbmgkcaappaeemecacaaaamgblgmobadaaaeemciacacaagmmgbl
obadacaeembbaaacaabllblbobadacaemiaeaaaaaalbgmaaobadaaaakibhacae
aalmmaecibacakalkiciacaeaamgblicmbaeadalkieoacafaabgpmmaibacaial
beahaaaaaabbmalbkbaaajafambiafaaaamgmggmobaaadadbeahaaaaaabebamg
oaafaaacamihacaaaamabalboaaaaeadmiahaaaaaamabaaaoaaaacaamiahiaad
aalemaaaoaabaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
Vector 465 [_WorldSpaceLightPos0]
Vector 464 [unity_4LightPosX0]
Vector 463 [unity_4LightPosY0]
Vector 462 [unity_4LightPosZ0]
Vector 461 [unity_4LightAtten0]
Vector 460 [unity_LightColor0]
Vector 459 [unity_LightColor1]
Vector 458 [unity_LightColor2]
Vector 457 [unity_LightColor3]
Vector 456 [unity_SHAr]
Vector 455 [unity_SHAg]
Vector 454 [unity_SHAb]
Vector 453 [unity_SHBr]
Vector 452 [unity_SHBg]
Vector 451 [unity_SHBb]
Vector 450 [unity_SHC]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 449 [unity_Scale]
Vector 448 [_MainTex_ST]
Vector 447 [_BumpMap_ST]
"sce_vp_rsx // 69 instructions using 9 registers
[Configuration]
8
0000004541050900
[Defaults]
1
446 3
000000003f8000003f000000
[Microcode]
1104
00009c6c005d100d8186c0836041fffc00039c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc00019c6c009c120c013fc0c36041dffc
401f9c6c011bf800810040d560607f9c401f9c6c011c0808010400d740619f9c
00011c6c01d0300d8106c0c360403ffc00011c6c01d0200d8106c0c360405ffc
00011c6c01d0100d8106c0c360409ffc00011c6c01d0000d8106c0c360411ffc
00019c6c01d0500d8106c0c360403ffc00021c6c01d0400d8106c0c360405ffc
00001c6c01d0600d8106c0c360403ffc00029c6c01d0a00d8286c0c360405ffc
00029c6c01d0900d8286c0c360409ffc00029c6c01d0800d8286c0c360411ffc
00021c6c0150400c068600c360411ffc00021c6c0150600c068600c360403ffc
00021c6c0150500c068600c360409ffc00031c6c0190a00c0086c0c360405ffc
00031c6c0190900c0086c0c360409ffc00031c6c0190800c0086c0c360411ffc
00001c6c00dce00d8186c0bfe021fffc00009c6c00dd000d8186c0b54221fffc
00019c6c00dcf00d8186c0bfe1a1fffc00041c6c00800243011847436041dffc
00039c6c01000230812187630421dffc401f9c6c0040000d8486c0836041ff80
401f9c6c004000558486c08360407fac00031c6c011c100c0cbfc0e30041dffc
00041c6c009be00e04aa80c36041dffc401f9c6c0140020c0106054360405fa4
00011c6c0080002a8886c3436041fffc00041c6c009d202a908000c360409ffc
00019c6c0080000d8686c3436041fffc00029c6c0080002a8895444360403ffc
00021c6c0040007f8886c08360405ffc00011c6c010000000886c1436121fffc
00009c6c0100000d8286c14361a1fffc401f9c6c00c000081086c09544219fac
00041c6c019c600c0886c0c360405ffc00041c6c019c700c0886c0c360409ffc
00041c6c019c800c0886c0c360411ffc00029c6c010000000880047fe2a03ffc
00019c6c0080000d089a04436041fffc00011c6c0100007f8886c0436121fffc
00001c6c0100000d8086c04360a1fffc00009c6c01dc300d8686c0c360405ffc
00009c6c01dc400d8686c0c360409ffc00009c6c01dc500d8686c0c360411ffc
00009c6c00c0000c1086c08300a1dffc00019c6c009c207f8a8600c36041dffc
00019c6c00c0000c0686c08300a1dffc401f9c6c21400e0c01060540003100a4
00039c6c20800e0c0ebfc08aa029c0fc00021c6c209cd00d8086c0d54025e0fc
00021c6c00dbe02a8186c0836221fffc401f9c6c2140020c0106065fe02240a0
401f9c6c11400e0c0c86008002310020401f9c6c1140000c0e86054aa2288024
401f9c6c1140000c0c8607554224802000009c6c1080000d8486c15fe223e07c
00009c6c029be00d828000c36041fffc00001c6c0080000d8286c0436041fffc
00009c6c009cb02a808600c36041dffc00009c6c011cc000008600c300a1dffc
00001c6c011ca055008600c300a1dffc00001c6c011c907f808600c30021dffc
401f9c6c00c0000c0686c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 32 [unity_4LightPosX0] 4
Vector 48 [unity_4LightPosY0] 4
Vector 64 [unity_4LightPosZ0] 4
Vector 80 [unity_4LightAtten0] 4
Vector 96 [unity_LightColor0] 4
Vector 112 [unity_LightColor1] 4
Vector 128 [unity_LightColor2] 4
Vector 144 [unity_LightColor3] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 70 instructions, 7 temp regs, 0 temp arrays:
// ALU 39 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedgjdhocceoipmpllkbdlfpkjpjahbfcpgabaaaaaanaalaaaaadaaaaaa
cmaaaaaapeaaaaaakmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefcbmakaaaaeaaaabaaihacaaaafjaaaaae
egiocaaaaaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaae
egiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
pccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaadhccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaacahaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaalaaaaaakgiocaaaaaaaaaaa
alaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaa
abaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaa
aaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaabdaaaaaa
dcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaa
diaaaaajhcaabaaaacaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaal
hcaabaaaacaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaacaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadgaaaaaf
icaabaaaabaaaaaaabeaaaaaaaaaiadpdiaaaaaihcaabaaaacaaaaaaegbcbaaa
acaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaadaaaaaafgafbaaa
acaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaacaaaaaaegiicaaa
adaaaaaaamaaaaaaagaabaaaacaaaaaaegaibaaaadaaaaaadcaaaaakhcaabaaa
abaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaacaaaaaaegadbaaaacaaaaaa
bbaaaaaibcaabaaaacaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaabaaaaaa
bbaaaaaiccaabaaaacaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaabaaaaaa
bbaaaaaiecaabaaaacaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaabaaaaaa
diaaaaahpcaabaaaadaaaaaajgacbaaaabaaaaaaegakbaaaabaaaaaabbaaaaai
bcaabaaaaeaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaadaaaaaabbaaaaai
ccaabaaaaeaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaadaaaaaabbaaaaai
ecaabaaaaeaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaadaaaaaaaaaaaaah
hcaabaaaacaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaahicaabaaa
abaaaaaabkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaakicaabaaaabaaaaaa
akaabaaaabaaaaaaakaabaaaabaaaaaadkaabaiaebaaaaaaabaaaaaadcaaaaak
hcaabaaaacaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaaabaaaaaaegacbaaa
acaaaaaadiaaaaaihcaabaaaadaaaaaafgbfbaaaaaaaaaaaegiccaaaadaaaaaa
anaaaaaadcaaaaakhcaabaaaadaaaaaaegiccaaaadaaaaaaamaaaaaaagbabaaa
aaaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaadaaaaaaegiccaaaadaaaaaa
aoaaaaaakgbkbaaaaaaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaadaaaaaa
egiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaaadaaaaaaaaaaaaaj
pcaabaaaaeaaaaaafgafbaiaebaaaaaaadaaaaaaegiocaaaacaaaaaaadaaaaaa
diaaaaahpcaabaaaafaaaaaafgafbaaaabaaaaaaegaobaaaaeaaaaaadiaaaaah
pcaabaaaaeaaaaaaegaobaaaaeaaaaaaegaobaaaaeaaaaaaaaaaaaajpcaabaaa
agaaaaaaagaabaiaebaaaaaaadaaaaaaegiocaaaacaaaaaaacaaaaaaaaaaaaaj
pcaabaaaadaaaaaakgakbaiaebaaaaaaadaaaaaaegiocaaaacaaaaaaaeaaaaaa
dcaaaaajpcaabaaaafaaaaaaegaobaaaagaaaaaaagaabaaaabaaaaaaegaobaaa
afaaaaaadcaaaaajpcaabaaaabaaaaaaegaobaaaadaaaaaakgakbaaaabaaaaaa
egaobaaaafaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaaagaaaaaaegaobaaa
agaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaaadaaaaaaegaobaaaadaaaaaa
egaobaaaadaaaaaaegaobaaaaeaaaaaaeeaaaaafpcaabaaaaeaaaaaaegaobaaa
adaaaaaadcaaaaanpcaabaaaadaaaaaaegaobaaaadaaaaaaegiocaaaacaaaaaa
afaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpaoaaaaakpcaabaaa
adaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpegaobaaaadaaaaaa
diaaaaahpcaabaaaabaaaaaaegaobaaaabaaaaaaegaobaaaaeaaaaaadeaaaaak
pcaabaaaabaaaaaaegaobaaaabaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadiaaaaahpcaabaaaabaaaaaaegaobaaaadaaaaaaegaobaaaabaaaaaa
diaaaaaihcaabaaaadaaaaaafgafbaaaabaaaaaaegiccaaaacaaaaaaahaaaaaa
dcaaaaakhcaabaaaadaaaaaaegiccaaaacaaaaaaagaaaaaaagaabaaaabaaaaaa
egacbaaaadaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaacaaaaaaaiaaaaaa
kgakbaaaabaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaa
acaaaaaaajaaaaaapgapbaaaabaaaaaaegacbaaaabaaaaaaaaaaaaahhccabaaa
aeaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaadiaaaaaiccaabaaaaaaaaaaa
bkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaa
agahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaaf
mccabaaaafaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaaafaaaaaakgakbaaa
abaaaaaamgaabaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
"!!GLES


#ifdef VERTEX

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_9;
  highp vec3 tmpvar_10;
  tmpvar_9 = tmpvar_1.xyz;
  tmpvar_10 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_11;
  tmpvar_11[0].x = tmpvar_9.x;
  tmpvar_11[0].y = tmpvar_10.x;
  tmpvar_11[0].z = tmpvar_2.x;
  tmpvar_11[1].x = tmpvar_9.y;
  tmpvar_11[1].y = tmpvar_10.y;
  tmpvar_11[1].z = tmpvar_2.y;
  tmpvar_11[2].x = tmpvar_9.z;
  tmpvar_11[2].y = tmpvar_10.z;
  tmpvar_11[2].z = tmpvar_2.z;
  highp vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_11 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_12;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_14;
  tmpvar_14.w = 1.0;
  tmpvar_14.xyz = tmpvar_8;
  mediump vec3 tmpvar_15;
  mediump vec4 normal_16;
  normal_16 = tmpvar_14;
  highp float vC_17;
  mediump vec3 x3_18;
  mediump vec3 x2_19;
  mediump vec3 x1_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAr, normal_16);
  x1_20.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAg, normal_16);
  x1_20.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHAb, normal_16);
  x1_20.z = tmpvar_23;
  mediump vec4 tmpvar_24;
  tmpvar_24 = (normal_16.xyzz * normal_16.yzzx);
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBr, tmpvar_24);
  x2_19.x = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBg, tmpvar_24);
  x2_19.y = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = dot (unity_SHBb, tmpvar_24);
  x2_19.z = tmpvar_27;
  mediump float tmpvar_28;
  tmpvar_28 = ((normal_16.x * normal_16.x) - (normal_16.y * normal_16.y));
  vC_17 = tmpvar_28;
  highp vec3 tmpvar_29;
  tmpvar_29 = (unity_SHC.xyz * vC_17);
  x3_18 = tmpvar_29;
  tmpvar_15 = ((x1_20 + x2_19) + x3_18);
  shlight_3 = tmpvar_15;
  tmpvar_6 = shlight_3;
  highp vec3 tmpvar_30;
  tmpvar_30 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_31;
  tmpvar_31 = (unity_4LightPosX0 - tmpvar_30.x);
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosY0 - tmpvar_30.y);
  highp vec4 tmpvar_33;
  tmpvar_33 = (unity_4LightPosZ0 - tmpvar_30.z);
  highp vec4 tmpvar_34;
  tmpvar_34 = (((tmpvar_31 * tmpvar_31) + (tmpvar_32 * tmpvar_32)) + (tmpvar_33 * tmpvar_33));
  highp vec4 tmpvar_35;
  tmpvar_35 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_31 * tmpvar_8.x) + (tmpvar_32 * tmpvar_8.y)) + (tmpvar_33 * tmpvar_8.z)) * inversesqrt(tmpvar_34))) * (1.0/((1.0 + (tmpvar_34 * unity_4LightAtten0)))));
  highp vec3 tmpvar_36;
  tmpvar_36 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_35.x) + (unity_LightColor[1].xyz * tmpvar_35.y)) + (unity_LightColor[2].xyz * tmpvar_35.z)) + (unity_LightColor[3].xyz * tmpvar_35.w)));
  tmpvar_6 = tmpvar_36;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_5;
  tmpvar_5 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), tmpvar_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  lowp float tmpvar_10;
  mediump float lightShadowDataX_11;
  highp float dist_12;
  lowp float tmpvar_13;
  tmpvar_13 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  dist_12 = tmpvar_13;
  highp float tmpvar_14;
  tmpvar_14 = _LightShadowData.x;
  lightShadowDataX_11 = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = max (float((dist_12 > (xlv_TEXCOORD4.z / xlv_TEXCOORD4.w))), lightShadowDataX_11);
  tmpvar_10 = tmpvar_15;
  highp vec3 tmpvar_16;
  tmpvar_16 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_17;
  lightDir_17 = xlv_TEXCOORD2;
  mediump vec3 viewDir_18;
  viewDir_18 = tmpvar_16;
  mediump float atten_19;
  atten_19 = tmpvar_10;
  mediump vec4 c_20;
  highp float nh_21;
  mediump float tmpvar_22;
  tmpvar_22 = max (0.0, dot (tmpvar_5, lightDir_17));
  mediump float tmpvar_23;
  tmpvar_23 = max (0.0, dot (tmpvar_5, normalize((lightDir_17 + viewDir_18))));
  nh_21 = tmpvar_23;
  mediump float tmpvar_24;
  tmpvar_24 = ((1.0 - dot (normalize(viewDir_18), tmpvar_5)) + dot (normalize(lightDir_17), tmpvar_5));
  highp vec3 tmpvar_25;
  tmpvar_25 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_22) + ((((_LightColor0.xyz * pow (nh_21, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * (atten_19 * 2.0));
  c_20.xyz = tmpvar_25;
  highp vec3 tmpvar_26;
  tmpvar_26 = (c_20.xyz + (c_20.xyz * (pow (tmpvar_24, _RimPower) * _RimColor.xyz)));
  c_20.xyz = tmpvar_26;
  c_20.w = tmpvar_9;
  c_1 = c_20;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
"!!GLES


#ifdef VERTEX

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (glstate_matrix_mvp * _glesVertex);
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_8;
  tmpvar_8[0] = _Object2World[0].xyz;
  tmpvar_8[1] = _Object2World[1].xyz;
  tmpvar_8[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_10;
  highp vec3 tmpvar_11;
  tmpvar_10 = tmpvar_1.xyz;
  tmpvar_11 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_12;
  tmpvar_12[0].x = tmpvar_10.x;
  tmpvar_12[0].y = tmpvar_11.x;
  tmpvar_12[0].z = tmpvar_2.x;
  tmpvar_12[1].x = tmpvar_10.y;
  tmpvar_12[1].y = tmpvar_11.y;
  tmpvar_12[1].z = tmpvar_2.y;
  tmpvar_12[2].x = tmpvar_10.z;
  tmpvar_12[2].y = tmpvar_11.z;
  tmpvar_12[2].z = tmpvar_2.z;
  highp vec3 tmpvar_13;
  tmpvar_13 = (tmpvar_12 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_13;
  highp vec4 tmpvar_14;
  tmpvar_14.w = 1.0;
  tmpvar_14.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_15;
  tmpvar_15.w = 1.0;
  tmpvar_15.xyz = tmpvar_9;
  mediump vec3 tmpvar_16;
  mediump vec4 normal_17;
  normal_17 = tmpvar_15;
  highp float vC_18;
  mediump vec3 x3_19;
  mediump vec3 x2_20;
  mediump vec3 x1_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAr, normal_17);
  x1_21.x = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHAg, normal_17);
  x1_21.y = tmpvar_23;
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHAb, normal_17);
  x1_21.z = tmpvar_24;
  mediump vec4 tmpvar_25;
  tmpvar_25 = (normal_17.xyzz * normal_17.yzzx);
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBr, tmpvar_25);
  x2_20.x = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = dot (unity_SHBg, tmpvar_25);
  x2_20.y = tmpvar_27;
  highp float tmpvar_28;
  tmpvar_28 = dot (unity_SHBb, tmpvar_25);
  x2_20.z = tmpvar_28;
  mediump float tmpvar_29;
  tmpvar_29 = ((normal_17.x * normal_17.x) - (normal_17.y * normal_17.y));
  vC_18 = tmpvar_29;
  highp vec3 tmpvar_30;
  tmpvar_30 = (unity_SHC.xyz * vC_18);
  x3_19 = tmpvar_30;
  tmpvar_16 = ((x1_21 + x2_20) + x3_19);
  shlight_3 = tmpvar_16;
  tmpvar_6 = shlight_3;
  highp vec3 tmpvar_31;
  tmpvar_31 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosX0 - tmpvar_31.x);
  highp vec4 tmpvar_33;
  tmpvar_33 = (unity_4LightPosY0 - tmpvar_31.y);
  highp vec4 tmpvar_34;
  tmpvar_34 = (unity_4LightPosZ0 - tmpvar_31.z);
  highp vec4 tmpvar_35;
  tmpvar_35 = (((tmpvar_32 * tmpvar_32) + (tmpvar_33 * tmpvar_33)) + (tmpvar_34 * tmpvar_34));
  highp vec4 tmpvar_36;
  tmpvar_36 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_32 * tmpvar_9.x) + (tmpvar_33 * tmpvar_9.y)) + (tmpvar_34 * tmpvar_9.z)) * inversesqrt(tmpvar_35))) * (1.0/((1.0 + (tmpvar_35 * unity_4LightAtten0)))));
  highp vec3 tmpvar_37;
  tmpvar_37 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_36.x) + (unity_LightColor[1].xyz * tmpvar_36.y)) + (unity_LightColor[2].xyz * tmpvar_36.z)) + (unity_LightColor[3].xyz * tmpvar_36.w)));
  tmpvar_6 = tmpvar_37;
  highp vec4 o_38;
  highp vec4 tmpvar_39;
  tmpvar_39 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_40;
  tmpvar_40.x = tmpvar_39.x;
  tmpvar_40.y = (tmpvar_39.y * _ProjectionParams.x);
  o_38.xy = (tmpvar_40 + tmpvar_39.w);
  o_38.zw = tmpvar_7.zw;
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_12 * (((_World2Object * tmpvar_14).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = o_38;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 normal_5;
  normal_5.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_5.z = sqrt((1.0 - clamp (dot (normal_5.xy, normal_5.xy), 0.0, 1.0)));
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), normal_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  lowp float tmpvar_10;
  tmpvar_10 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  highp vec3 tmpvar_11;
  tmpvar_11 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_12;
  lightDir_12 = xlv_TEXCOORD2;
  mediump vec3 viewDir_13;
  viewDir_13 = tmpvar_11;
  mediump float atten_14;
  atten_14 = tmpvar_10;
  mediump vec4 c_15;
  highp float nh_16;
  mediump float tmpvar_17;
  tmpvar_17 = max (0.0, dot (normal_5, lightDir_12));
  mediump float tmpvar_18;
  tmpvar_18 = max (0.0, dot (normal_5, normalize((lightDir_12 + viewDir_13))));
  nh_16 = tmpvar_18;
  mediump float tmpvar_19;
  tmpvar_19 = ((1.0 - dot (normalize(viewDir_13), normal_5)) + dot (normalize(lightDir_12), normal_5));
  highp vec3 tmpvar_20;
  tmpvar_20 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_17) + ((((_LightColor0.xyz * pow (nh_16, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * (atten_14 * 2.0));
  c_15.xyz = tmpvar_20;
  highp vec3 tmpvar_21;
  tmpvar_21 = (c_15.xyz + (c_15.xyz * (pow (tmpvar_19, _RimPower) * _RimColor.xyz)));
  c_15.xyz = tmpvar_21;
  c_15.w = tmpvar_9;
  c_1 = c_15;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_WorldSpaceLightPos0]
Vector 15 [unity_4LightPosX0]
Vector 16 [unity_4LightPosY0]
Vector 17 [unity_4LightPosZ0]
Vector 18 [unity_4LightAtten0]
Vector 19 [unity_LightColor0]
Vector 20 [unity_LightColor1]
Vector 21 [unity_LightColor2]
Vector 22 [unity_LightColor3]
Vector 23 [unity_SHAr]
Vector 24 [unity_SHAg]
Vector 25 [unity_SHAb]
Vector 26 [unity_SHBr]
Vector 27 [unity_SHBg]
Vector 28 [unity_SHBb]
Vector 29 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 30 [unity_Scale]
Vector 31 [unity_NPOTScale]
Vector 32 [_MainTex_ST]
Vector 33 [_BumpMap_ST]
"agal_vs
c34 1.0 0.0 0.5 0.0
[bc]
adaaaaaaadaaahacabaaaaoeaaaaaaaaboaaaappabaaaaaa mul r3.xyz, a1, c30.w
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.x, a0, c5
bfaaaaaaabaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.x, r0.x
abaaaaaaabaaapacabaaaaaaacaaaaaabaaaaaoeabaaaaaa add r1, r1.x, c16
bcaaaaaaadaaaiacadaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r3.w, r3.xyzz, c5
bcaaaaaaaeaaabacadaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r4.x, r3.xyzz, c4
bcaaaaaaadaaabacadaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r3.x, r3.xyzz, c6
adaaaaaaacaaapacadaaaappacaaaaaaabaaaaoeacaaaaaa mul r2, r3.w, r1
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaapacaaaaaaaaacaaaaaaapaaaaoeabaaaaaa add r0, r0.x, c15
adaaaaaaabaaapacabaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r1, r1, r1
aaaaaaaaaeaaaeacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r4.z, r3.x
adaaaaaaafaaapacaeaaaaaaacaaaaaaaaaaaaoeacaaaaaa mul r5, r4.x, r0
abaaaaaaacaaapacafaaaaoeacaaaaaaacaaaaoeacaaaaaa add r2, r5, r2
aaaaaaaaaeaaaiacccaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r4.w, c34.x
bdaaaaaaaeaaacacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r4.y, a0, c6
adaaaaaaafaaapacaaaaaaoeacaaaaaaaaaaaaoeacaaaaaa mul r5, r0, r0
abaaaaaaabaaapacafaaaaoeacaaaaaaabaaaaoeacaaaaaa add r1, r5, r1
bfaaaaaaaaaaacacaeaaaaffacaaaaaaaaaaaaaaaaaaaaaa neg r0.y, r4.y
abaaaaaaaaaaapacaaaaaaffacaaaaaabbaaaaoeabaaaaaa add r0, r0.y, c17
adaaaaaaafaaapacaaaaaaoeacaaaaaaaaaaaaoeacaaaaaa mul r5, r0, r0
abaaaaaaabaaapacafaaaaoeacaaaaaaabaaaaoeacaaaaaa add r1, r5, r1
adaaaaaaaaaaapacadaaaaaaacaaaaaaaaaaaaoeacaaaaaa mul r0, r3.x, r0
abaaaaaaaaaaapacaaaaaaoeacaaaaaaacaaaaoeacaaaaaa add r0, r0, r2
adaaaaaaacaaapacabaaaaoeacaaaaaabcaaaaoeabaaaaaa mul r2, r1, c18
aaaaaaaaaeaaacacadaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r4.y, r3.w
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
akaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rsq r1.y, r1.y
akaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rsq r1.w, r1.w
akaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rsq r1.z, r1.z
adaaaaaaaaaaapacaaaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r0, r0, r1
abaaaaaaabaaapacacaaaaoeacaaaaaaccaaaaaaabaaaaaa add r1, r2, c34.x
bdaaaaaaacaaaeacaeaaaaoeacaaaaaabjaaaaoeabaaaaaa dp4 r2.z, r4, c25
bdaaaaaaacaaacacaeaaaaoeacaaaaaabiaaaaoeabaaaaaa dp4 r2.y, r4, c24
bdaaaaaaacaaabacaeaaaaoeacaaaaaabhaaaaoeabaaaaaa dp4 r2.x, r4, c23
afaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r1.x, r1.x
afaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rcp r1.y, r1.y
afaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rcp r1.w, r1.w
afaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rcp r1.z, r1.z
ahaaaaaaaaaaapacaaaaaaoeacaaaaaaccaaaaffabaaaaaa max r0, r0, c34.y
adaaaaaaaaaaapacaaaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r0, r0, r1
adaaaaaaabaaahacaaaaaaffacaaaaaabeaaaaoeabaaaaaa mul r1.xyz, r0.y, c20
adaaaaaaafaaahacaaaaaaaaacaaaaaabdaaaaoeabaaaaaa mul r5.xyz, r0.x, c19
abaaaaaaabaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r5.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakkacaaaaaabfaaaaoeabaaaaaa mul r0.xyz, r0.z, c21
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaabaaahacaaaaaappacaaaaaabgaaaaoeabaaaaaa mul r1.xyz, r0.w, c22
abaaaaaaabaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa add r1.xyz, r1.xyzz, r0.xyzz
adaaaaaaaaaaapacaeaaaakeacaaaaaaaeaaaacjacaaaaaa mul r0, r4.xyzz, r4.yzzx
adaaaaaaabaaaiacadaaaappacaaaaaaadaaaappacaaaaaa mul r1.w, r3.w, r3.w
bdaaaaaaadaaaeacaaaaaaoeacaaaaaabmaaaaoeabaaaaaa dp4 r3.z, r0, c28
bdaaaaaaadaaacacaaaaaaoeacaaaaaablaaaaoeabaaaaaa dp4 r3.y, r0, c27
bdaaaaaaadaaabacaaaaaaoeacaaaaaabkaaaaoeabaaaaaa dp4 r3.x, r0, c26
adaaaaaaafaaaiacaeaaaaaaacaaaaaaaeaaaaaaacaaaaaa mul r5.w, r4.x, r4.x
acaaaaaaabaaaiacafaaaappacaaaaaaabaaaappacaaaaaa sub r1.w, r5.w, r1.w
adaaaaaaaaaaahacabaaaappacaaaaaabnaaaaoeabaaaaaa mul r0.xyz, r1.w, c29
abaaaaaaacaaahacacaaaakeacaaaaaaadaaaakeacaaaaaa add r2.xyz, r2.xyzz, r3.xyzz
abaaaaaaacaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa add r2.xyz, r2.xyzz, r0.xyzz
abaaaaaaadaaahaeacaaaakeacaaaaaaabaaaakeacaaaaaa add v3.xyz, r2.xyzz, r1.xyzz
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaafaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r5.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r5.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c14, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c14, r0
aaaaaaaaabaaaiacccaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c34.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 r0.w, a0, c3
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 r0.z, a0, c2
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaafaaahacacaaaakeacaaaaaaboaaaappabaaaaaa mul r5.xyz, r2.xyzz, c30.w
acaaaaaaacaaahacafaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r5.xyzz, a0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacaoaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c14, r1
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 r0.x, a0, c0
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 r0.y, a0, c1
adaaaaaaabaaahacaaaaaapeacaaaaaaccaaaakkabaaaaaa mul r1.xyz, r0.xyww, c34.z
adaaaaaaabaaacacabaaaaffacaaaaaaanaaaaaaabaaaaaa mul r1.y, r1.y, c13.x
abaaaaaaabaaadacabaaaafeacaaaaaaabaaaakkacaaaaaa add r1.xy, r1.xyyy, r1.z
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
adaaaaaaaeaaadaeabaaaafeacaaaaaabpaaaaoeabaaaaaa mul v4.xy, r1.xyyy, c31
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
aaaaaaaaaeaaamaeaaaaaaopacaaaaaaaaaaaaaaaaaaaaaa mov v4.zw, r0.wwzw
adaaaaaaafaaamacadaaaaeeaaaaaaaacbaaaaeeabaaaaaa mul r5.zw, a3.xyxy, c33.xyxy
abaaaaaaaaaaamaeafaaaaopacaaaaaacbaaaaoeabaaaaaa add v0.zw, r5.wwzw, c33
adaaaaaaafaaadacadaaaaoeaaaaaaaacaaaaaoeabaaaaaa mul r5.xy, a3, c32
abaaaaaaaaaaadaeafaaaafeacaaaaaacaaaaaooabaaaaaa add v0.xy, r5.xyyy, c32.zwzw
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 32 [unity_4LightPosX0] 4
Vector 48 [unity_4LightPosY0] 4
Vector 64 [unity_4LightPosZ0] 4
Vector 80 [unity_4LightAtten0] 4
Vector 96 [unity_LightColor0] 4
Vector 112 [unity_LightColor1] 4
Vector 128 [unity_LightColor2] 4
Vector 144 [unity_LightColor3] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 70 instructions, 7 temp regs, 0 temp arrays:
// ALU 39 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedomcjmppfekaeodnckdcbgnpgcbkgdppjabaaaaaaombbaaaaaeaaaaaa
daaaaaaaeiagaaaagmbaaaaadebbaaaaebgpgodjbaagaaaabaagaaaaaaacpopp
jeafaaaahmaaaaaaahaaceaaaaaahiaaaaaahiaaaaaaceaaabaahiaaaaaaakaa
acaaabaaaaaaaaaaabaaaeaaacaaadaaaaaaaaaaacaaaaaaabaaafaaaaaaaaaa
acaaacaaaiaaagaaaaaaaaaaacaabcaaahaaaoaaaaaaaaaaadaaaaaaaeaabfaa
aaaaaaaaadaaamaaajaabjaaaaaaaaaaaaaaaaaaaaacpoppfbaaaaafccaaapka
aaaaiadpaaaaaaaaaaaaaadpaaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaac
afaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapja
aeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaaeaaaaaeaaaaamoaadaaeeja
acaaeekaacaaoekaabaaaaacaaaaapiaafaaoekaafaaaaadabaaahiaaaaaffia
boaaoekaaeaaaaaeabaaahiabnaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahia
bpaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiacaaaoekaaaaappiaaaaaoeia
aiaaaaadacaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaacaaoejaafaaaaad
acaaahiaabaanciaabaamjjaaeaaaaaeabaaahiaabaamjiaabaancjaacaaoeib
afaaaaadabaaahiaabaaoeiaabaappjaaiaaaaadacaaacoaabaaoeiaaaaaoeia
aiaaaaadacaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaadaaoekaafaaaaad
acaaahiaaaaaffiaboaaoekaaeaaaaaeaaaaaliabnaakekaaaaaaaiaacaakeia
aeaaaaaeaaaaahiabpaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeia
caaaoekaaeaaaaaeaaaaahiaaaaaoeiacbaappkaaaaaoejbaiaaaaadabaaaboa
abaaoejaaaaaoeiaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoa
acaaoejaaaaaoeiaafaaaaadaaaaahiaaaaaffjabkaaoekaaeaaaaaeaaaaahia
bjaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaahiablaaoekaaaaakkjaaaaaoeia
aeaaaaaeaaaaahiabmaaoekaaaaappjaaaaaoeiaacaaaaadabaaapiaaaaakkib
aiaaoekaacaaaaadacaaapiaaaaaaaibagaaoekaacaaaaadaaaaapiaaaaaffib
ahaaoekaafaaaaadadaaahiaacaaoejacbaappkaafaaaaadaeaaahiaadaaffia
bkaaoekaaeaaaaaeadaaaliabjaakekaadaaaaiaaeaakeiaaeaaaaaeadaaahia
blaaoekaadaakkiaadaapeiaafaaaaadaeaaapiaaaaaoeiaadaaffiaafaaaaad
aaaaapiaaaaaoeiaaaaaoeiaaeaaaaaeaaaaapiaacaaoeiaacaaoeiaaaaaoeia
aeaaaaaeacaaapiaacaaoeiaadaaaaiaaeaaoeiaaeaaaaaeacaaapiaabaaoeia
adaakkiaacaaoeiaaeaaaaaeaaaaapiaabaaoeiaabaaoeiaaaaaoeiaahaaaaac
abaaabiaaaaaaaiaahaaaaacabaaaciaaaaaffiaahaaaaacabaaaeiaaaaakkia
ahaaaaacabaaaiiaaaaappiaabaaaaacaeaaabiaccaaaakaaeaaaaaeaaaaapia
aaaaoeiaajaaoekaaeaaaaiaafaaaaadabaaapiaabaaoeiaacaaoeiaalaaaaad
abaaapiaabaaoeiaccaaffkaagaaaaacacaaabiaaaaaaaiaagaaaaacacaaacia
aaaaffiaagaaaaacacaaaeiaaaaakkiaagaaaaacacaaaiiaaaaappiaafaaaaad
aaaaapiaabaaoeiaacaaoeiaafaaaaadabaaahiaaaaaffiaalaaoekaaeaaaaae
abaaahiaakaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiaamaaoekaaaaakkia
abaaoeiaaeaaaaaeaaaaahiaanaaoekaaaaappiaaaaaoeiaabaaaaacadaaaiia
ccaaaakaajaaaaadabaaabiaaoaaoekaadaaoeiaajaaaaadabaaaciaapaaoeka
adaaoeiaajaaaaadabaaaeiabaaaoekaadaaoeiaafaaaaadacaaapiaadaacjia
adaakeiaajaaaaadaeaaabiabbaaoekaacaaoeiaajaaaaadaeaaaciabcaaoeka
acaaoeiaajaaaaadaeaaaeiabdaaoekaacaaoeiaacaaaaadabaaahiaabaaoeia
aeaaoeiaafaaaaadaaaaaiiaadaaffiaadaaffiaaeaaaaaeaaaaaiiaadaaaaia
adaaaaiaaaaappibaeaaaaaeabaaahiabeaaoekaaaaappiaabaaoeiaacaaaaad
adaaahoaaaaaoeiaabaaoeiaafaaaaadaaaaapiaaaaaffjabgaaoekaaeaaaaae
aaaaapiabfaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiabhaaoekaaaaakkja
aaaaoeiaaeaaaaaeaaaaapiabiaaoekaaaaappjaaaaaoeiaafaaaaadabaaabia
aaaaffiaaeaaaakaafaaaaadabaaaiiaabaaaaiaccaakkkaafaaaaadabaaafia
aaaapeiaccaakkkaacaaaaadaeaaadoaabaakkiaabaaomiaaeaaaaaeaaaaadma
aaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiaabaaaaacaeaaamoa
aaaaoeiappppaaaafdeieefcbmakaaaaeaaaabaaihacaaaafjaaaaaeegiocaaa
aaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaa
acaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaa
aaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaad
dcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaa
abaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaad
hccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaacahaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaa
aaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaadcaaaaalmccabaaaabaaaaaa
agbebaaaadaaaaaaagiecaaaaaaaaaaaalaaaaaakgiocaaaaaaaaaaaalaaaaaa
diaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
abaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaaaaaaaaai
hcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadiaaaaaj
hcaabaaaacaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaa
dcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaa
aaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaa
bcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaa
acaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaa
acaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaah
eccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadgaaaaaficaabaaa
abaaaaaaabeaaaaaaaaaiadpdiaaaaaihcaabaaaacaaaaaaegbcbaaaacaaaaaa
pgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaadaaaaaafgafbaaaacaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaacaaaaaaegiicaaaadaaaaaa
amaaaaaaagaabaaaacaaaaaaegaibaaaadaaaaaadcaaaaakhcaabaaaabaaaaaa
egiccaaaadaaaaaaaoaaaaaakgakbaaaacaaaaaaegadbaaaacaaaaaabbaaaaai
bcaabaaaacaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaabaaaaaabbaaaaai
ccaabaaaacaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaabaaaaaabbaaaaai
ecaabaaaacaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaabaaaaaadiaaaaah
pcaabaaaadaaaaaajgacbaaaabaaaaaaegakbaaaabaaaaaabbaaaaaibcaabaaa
aeaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaadaaaaaabbaaaaaiccaabaaa
aeaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaadaaaaaabbaaaaaiecaabaaa
aeaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaadaaaaaaaaaaaaahhcaabaaa
acaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaahicaabaaaabaaaaaa
bkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaakicaabaaaabaaaaaaakaabaaa
abaaaaaaakaabaaaabaaaaaadkaabaiaebaaaaaaabaaaaaadcaaaaakhcaabaaa
acaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaaabaaaaaaegacbaaaacaaaaaa
diaaaaaihcaabaaaadaaaaaafgbfbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaa
dcaaaaakhcaabaaaadaaaaaaegiccaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaa
egacbaaaadaaaaaadcaaaaakhcaabaaaadaaaaaaegiccaaaadaaaaaaaoaaaaaa
kgbkbaaaaaaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaadaaaaaaegiccaaa
adaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaaadaaaaaaaaaaaaajpcaabaaa
aeaaaaaafgafbaiaebaaaaaaadaaaaaaegiocaaaacaaaaaaadaaaaaadiaaaaah
pcaabaaaafaaaaaafgafbaaaabaaaaaaegaobaaaaeaaaaaadiaaaaahpcaabaaa
aeaaaaaaegaobaaaaeaaaaaaegaobaaaaeaaaaaaaaaaaaajpcaabaaaagaaaaaa
agaabaiaebaaaaaaadaaaaaaegiocaaaacaaaaaaacaaaaaaaaaaaaajpcaabaaa
adaaaaaakgakbaiaebaaaaaaadaaaaaaegiocaaaacaaaaaaaeaaaaaadcaaaaaj
pcaabaaaafaaaaaaegaobaaaagaaaaaaagaabaaaabaaaaaaegaobaaaafaaaaaa
dcaaaaajpcaabaaaabaaaaaaegaobaaaadaaaaaakgakbaaaabaaaaaaegaobaaa
afaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaaagaaaaaaegaobaaaagaaaaaa
egaobaaaaeaaaaaadcaaaaajpcaabaaaadaaaaaaegaobaaaadaaaaaaegaobaaa
adaaaaaaegaobaaaaeaaaaaaeeaaaaafpcaabaaaaeaaaaaaegaobaaaadaaaaaa
dcaaaaanpcaabaaaadaaaaaaegaobaaaadaaaaaaegiocaaaacaaaaaaafaaaaaa
aceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpaoaaaaakpcaabaaaadaaaaaa
aceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpegaobaaaadaaaaaadiaaaaah
pcaabaaaabaaaaaaegaobaaaabaaaaaaegaobaaaaeaaaaaadeaaaaakpcaabaaa
abaaaaaaegaobaaaabaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
diaaaaahpcaabaaaabaaaaaaegaobaaaadaaaaaaegaobaaaabaaaaaadiaaaaai
hcaabaaaadaaaaaafgafbaaaabaaaaaaegiccaaaacaaaaaaahaaaaaadcaaaaak
hcaabaaaadaaaaaaegiccaaaacaaaaaaagaaaaaaagaabaaaabaaaaaaegacbaaa
adaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaacaaaaaaaiaaaaaakgakbaaa
abaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaacaaaaaa
ajaaaaaapgapbaaaabaaaaaaegacbaaaabaaaaaaaaaaaaahhccabaaaaeaaaaaa
egacbaaaabaaaaaaegacbaaaacaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaa
aaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaa
aaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaa
afaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaaafaaaaaakgakbaaaabaaaaaa
mgaabaaaabaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform sampler2D _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 433
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 95
highp vec3 Shade4PointLights( in highp vec4 lightPosX, in highp vec4 lightPosY, in highp vec4 lightPosZ, in highp vec3 lightColor0, in highp vec3 lightColor1, in highp vec3 lightColor2, in highp vec3 lightColor3, in highp vec4 lightAttenSq, in highp vec3 pos, in highp vec3 normal ) {
    highp vec4 toLightX = (lightPosX - pos.x);
    highp vec4 toLightY = (lightPosY - pos.y);
    #line 99
    highp vec4 toLightZ = (lightPosZ - pos.z);
    highp vec4 lengthSq = vec4( 0.0);
    lengthSq += (toLightX * toLightX);
    lengthSq += (toLightY * toLightY);
    #line 103
    lengthSq += (toLightZ * toLightZ);
    highp vec4 ndotl = vec4( 0.0);
    ndotl += (toLightX * normal.x);
    ndotl += (toLightY * normal.y);
    #line 107
    ndotl += (toLightZ * normal.z);
    highp vec4 corr = inversesqrt(lengthSq);
    ndotl = max( vec4( 0.0, 0.0, 0.0, 0.0), (ndotl * corr));
    highp vec4 atten = (1.0 / (1.0 + (lengthSq * lightAttenSq)));
    #line 111
    highp vec4 diff = (ndotl * atten);
    highp vec3 col = vec3( 0.0);
    col += (lightColor0 * diff.x);
    col += (lightColor1 * diff.y);
    #line 115
    col += (lightColor2 * diff.z);
    col += (lightColor3 * diff.w);
    return col;
}
#line 136
mediump vec3 ShadeSH9( in mediump vec4 normal ) {
    mediump vec3 x1;
    mediump vec3 x2;
    mediump vec3 x3;
    x1.x = dot( unity_SHAr, normal);
    #line 140
    x1.y = dot( unity_SHAg, normal);
    x1.z = dot( unity_SHAb, normal);
    mediump vec4 vB = (normal.xyzz * normal.yzzx);
    x2.x = dot( unity_SHBr, vB);
    #line 144
    x2.y = dot( unity_SHBg, vB);
    x2.z = dot( unity_SHBb, vB);
    highp float vC = ((normal.x * normal.x) - (normal.y * normal.y));
    x3 = (unity_SHC.xyz * vC);
    #line 148
    return ((x1 + x2) + x3);
}
#line 435
v2f_surf vert_surf( in appdata_full v ) {
    #line 437
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 441
    highp vec3 worldN = (mat3( _Object2World) * (v.normal * unity_Scale.w));
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    #line 445
    o.lightDir = lightDir;
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    highp vec3 shlight = ShadeSH9( vec4( worldN, 1.0));
    #line 449
    o.vlight = shlight;
    highp vec3 worldPos = (_Object2World * v.vertex).xyz;
    o.vlight += Shade4PointLights( unity_4LightPosX0, unity_4LightPosY0, unity_4LightPosZ0, unity_LightColor[0].xyz, unity_LightColor[1].xyz, unity_LightColor[2].xyz, unity_LightColor[3].xyz, unity_4LightAtten0, worldPos, worldN);
    o._ShadowCoord = (unity_World2Shadow[0] * (_Object2World * v.vertex));
    #line 454
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out highp vec3 xlv_TEXCOORD1;
out lowp vec3 xlv_TEXCOORD2;
out lowp vec3 xlv_TEXCOORD3;
out highp vec4 xlv_TEXCOORD4;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD2 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD3 = vec3(xl_retval.vlight);
    xlv_TEXCOORD4 = vec4(xl_retval._ShadowCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform sampler2D _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 433
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 395
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    #line 397
    mediump vec3 h = normalize((lightDir + viewDir));
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    #line 401
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    #line 405
    c.w = s.Alpha;
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 415
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 419
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 384
lowp float unitySampleShadow( in highp vec4 shadowCoord ) {
    highp float dist = textureProj( _ShadowMapTexture, shadowCoord).x;
    mediump float lightShadowDataX = _LightShadowData.x;
    #line 388
    return max( float((dist > (shadowCoord.z / shadowCoord.w))), lightShadowDataX);
}
#line 456
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 458
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    surfIN.viewDir = IN.viewDir;
    #line 462
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    #line 466
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp float atten = unitySampleShadow( IN._ShadowCoord);
    #line 470
    lowp vec4 c = vec4( 0.0);
    c = LightingSimpleSpecular( o, IN.lightDir, normalize(IN.viewDir), atten);
    c.xyz += (o.Albedo * IN.vlight);
    c.xyz += o.Emission;
    #line 474
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in highp vec3 xlv_TEXCOORD1;
in lowp vec3 xlv_TEXCOORD2;
in lowp vec3 xlv_TEXCOORD3;
in highp vec4 xlv_TEXCOORD4;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD1);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD2);
    xlt_IN.vlight = vec3(xlv_TEXCOORD3);
    xlt_IN._ShadowCoord = vec4(xlv_TEXCOORD4);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "SHADOWS_NATIVE" }
"!!GLES


#ifdef VERTEX

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_8 = tmpvar_1.xyz;
  tmpvar_9 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_10;
  tmpvar_10[0].x = tmpvar_8.x;
  tmpvar_10[0].y = tmpvar_9.x;
  tmpvar_10[0].z = tmpvar_2.x;
  tmpvar_10[1].x = tmpvar_8.y;
  tmpvar_10[1].y = tmpvar_9.y;
  tmpvar_10[1].z = tmpvar_2.y;
  tmpvar_10[2].x = tmpvar_8.z;
  tmpvar_10[2].y = tmpvar_9.z;
  tmpvar_10[2].z = tmpvar_2.z;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_11;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_14;
  mediump vec4 normal_15;
  normal_15 = tmpvar_13;
  highp float vC_16;
  mediump vec3 x3_17;
  mediump vec3 x2_18;
  mediump vec3 x1_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAr, normal_15);
  x1_19.x = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAg, normal_15);
  x1_19.y = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAb, normal_15);
  x1_19.z = tmpvar_22;
  mediump vec4 tmpvar_23;
  tmpvar_23 = (normal_15.xyzz * normal_15.yzzx);
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBr, tmpvar_23);
  x2_18.x = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBg, tmpvar_23);
  x2_18.y = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBb, tmpvar_23);
  x2_18.z = tmpvar_26;
  mediump float tmpvar_27;
  tmpvar_27 = ((normal_15.x * normal_15.x) - (normal_15.y * normal_15.y));
  vC_16 = tmpvar_27;
  highp vec3 tmpvar_28;
  tmpvar_28 = (unity_SHC.xyz * vC_16);
  x3_17 = tmpvar_28;
  tmpvar_14 = ((x1_19 + x2_18) + x3_17);
  shlight_3 = tmpvar_14;
  tmpvar_6 = shlight_3;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform lowp sampler2DShadow _ShadowMapTexture;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_5;
  tmpvar_5 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), tmpvar_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  lowp float shadow_10;
  lowp float tmpvar_11;
  tmpvar_11 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD4.xyz);
  highp float tmpvar_12;
  tmpvar_12 = (_LightShadowData.x + (tmpvar_11 * (1.0 - _LightShadowData.x)));
  shadow_10 = tmpvar_12;
  highp vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_14;
  lightDir_14 = xlv_TEXCOORD2;
  mediump vec3 viewDir_15;
  viewDir_15 = tmpvar_13;
  mediump float atten_16;
  atten_16 = shadow_10;
  mediump vec4 c_17;
  highp float nh_18;
  mediump float tmpvar_19;
  tmpvar_19 = max (0.0, dot (tmpvar_5, lightDir_14));
  mediump float tmpvar_20;
  tmpvar_20 = max (0.0, dot (tmpvar_5, normalize((lightDir_14 + viewDir_15))));
  nh_18 = tmpvar_20;
  mediump float tmpvar_21;
  tmpvar_21 = ((1.0 - dot (normalize(viewDir_15), tmpvar_5)) + dot (normalize(lightDir_14), tmpvar_5));
  highp vec3 tmpvar_22;
  tmpvar_22 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_19) + ((((_LightColor0.xyz * pow (nh_18, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * (atten_16 * 2.0));
  c_17.xyz = tmpvar_22;
  highp vec3 tmpvar_23;
  tmpvar_23 = (c_17.xyz + (c_17.xyz * (pow (tmpvar_21, _RimPower) * _RimColor.xyz)));
  c_17.xyz = tmpvar_23;
  c_17.w = tmpvar_9;
  c_1 = c_17;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "SHADOWS_NATIVE" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform lowp sampler2DShadow _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 433
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 454
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 136
mediump vec3 ShadeSH9( in mediump vec4 normal ) {
    mediump vec3 x1;
    mediump vec3 x2;
    mediump vec3 x3;
    x1.x = dot( unity_SHAr, normal);
    #line 140
    x1.y = dot( unity_SHAg, normal);
    x1.z = dot( unity_SHAb, normal);
    mediump vec4 vB = (normal.xyzz * normal.yzzx);
    x2.x = dot( unity_SHBr, vB);
    #line 144
    x2.y = dot( unity_SHBg, vB);
    x2.z = dot( unity_SHBb, vB);
    highp float vC = ((normal.x * normal.x) - (normal.y * normal.y));
    x3 = (unity_SHC.xyz * vC);
    #line 148
    return ((x1 + x2) + x3);
}
#line 435
v2f_surf vert_surf( in appdata_full v ) {
    #line 437
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 441
    highp vec3 worldN = (mat3( _Object2World) * (v.normal * unity_Scale.w));
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    #line 445
    o.lightDir = lightDir;
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    highp vec3 shlight = ShadeSH9( vec4( worldN, 1.0));
    #line 449
    o.vlight = shlight;
    o._ShadowCoord = (unity_World2Shadow[0] * (_Object2World * v.vertex));
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out highp vec3 xlv_TEXCOORD1;
out lowp vec3 xlv_TEXCOORD2;
out lowp vec3 xlv_TEXCOORD3;
out highp vec4 xlv_TEXCOORD4;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD2 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD3 = vec3(xl_retval.vlight);
    xlv_TEXCOORD4 = vec4(xl_retval._ShadowCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_shadow2D(mediump sampler2DShadow s, vec3 coord) { return texture (s, coord); }
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform lowp sampler2DShadow _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 433
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 454
#line 395
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    #line 397
    mediump vec3 h = normalize((lightDir + viewDir));
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    #line 401
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    #line 405
    c.w = s.Alpha;
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 415
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 419
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 384
lowp float unitySampleShadow( in highp vec4 shadowCoord ) {
    lowp float shadow = xll_shadow2D( _ShadowMapTexture, shadowCoord.xyz.xyz);
    shadow = (_LightShadowData.x + (shadow * (1.0 - _LightShadowData.x)));
    #line 388
    return shadow;
}
#line 454
lowp vec4 frag_surf( in v2f_surf IN ) {
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    #line 458
    surfIN.uv_BumpMap = IN.pack0.zw;
    surfIN.viewDir = IN.viewDir;
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    #line 462
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    #line 466
    surf( surfIN, o);
    lowp float atten = unitySampleShadow( IN._ShadowCoord);
    lowp vec4 c = vec4( 0.0);
    c = LightingSimpleSpecular( o, IN.lightDir, normalize(IN.viewDir), atten);
    #line 470
    c.xyz += (o.Albedo * IN.vlight);
    c.xyz += o.Emission;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in highp vec3 xlv_TEXCOORD1;
in lowp vec3 xlv_TEXCOORD2;
in lowp vec3 xlv_TEXCOORD3;
in highp vec4 xlv_TEXCOORD4;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD1);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD2);
    xlt_IN.vlight = vec3(xlv_TEXCOORD3);
    xlt_IN._ShadowCoord = vec4(xlv_TEXCOORD4);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "SHADOWS_NATIVE" }
"!!GLES


#ifdef VERTEX

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  tmpvar_4 = tmpvar_1.xyz;
  tmpvar_5 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_6;
  tmpvar_6[0].x = tmpvar_4.x;
  tmpvar_6[0].y = tmpvar_5.x;
  tmpvar_6[0].z = tmpvar_2.x;
  tmpvar_6[1].x = tmpvar_4.y;
  tmpvar_6[1].y = tmpvar_5.y;
  tmpvar_6[1].z = tmpvar_2.y;
  tmpvar_6[2].x = tmpvar_4.z;
  tmpvar_6[2].y = tmpvar_5.z;
  tmpvar_6[2].z = tmpvar_2.z;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = _WorldSpaceCameraPos;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_7).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp sampler2DShadow _ShadowMapTexture;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec3 tmpvar_4;
  tmpvar_4 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  highp float tmpvar_5;
  tmpvar_5 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), tmpvar_4), 0.0, 1.0));
  rim_3 = tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_6 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_6;
  lowp float shadow_7;
  lowp float tmpvar_8;
  tmpvar_8 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD3.xyz);
  highp float tmpvar_9;
  tmpvar_9 = (_LightShadowData.x + (tmpvar_8 * (1.0 - _LightShadowData.x)));
  shadow_7 = tmpvar_9;
  c_1.xyz = (texture2D (_MainTex, xlv_TEXCOORD0.xy).xyz * min ((2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD2).xyz), vec3((shadow_7 * 2.0))));
  c_1.w = texture2D (_MainTex, xlv_TEXCOORD0.xy).w;
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "SHADOWS_NATIVE" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    highp vec2 lmap;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform lowp sampler2DShadow _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 432
uniform highp vec4 unity_LightmapST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
uniform sampler2D unity_Lightmap;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 435
v2f_surf vert_surf( in appdata_full v ) {
    #line 437
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 441
    o.lmap.xy = ((v.texcoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
    highp vec3 worldN = (mat3( _Object2World) * (v.normal * unity_Scale.w));
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    #line 445
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    o._ShadowCoord = (unity_World2Shadow[0] * (_Object2World * v.vertex));
    #line 450
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out highp vec3 xlv_TEXCOORD1;
out highp vec2 xlv_TEXCOORD2;
out highp vec4 xlv_TEXCOORD3;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD2 = vec2(xl_retval.lmap);
    xlv_TEXCOORD3 = vec4(xl_retval._ShadowCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_shadow2D(mediump sampler2DShadow s, vec3 coord) { return texture (s, coord); }
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    highp vec2 lmap;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform lowp sampler2DShadow _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 432
uniform highp vec4 unity_LightmapST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
uniform sampler2D unity_Lightmap;
#line 176
lowp vec3 DecodeLightmap( in lowp vec4 color ) {
    #line 178
    return (2.0 * color.xyz);
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 415
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 419
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 384
lowp float unitySampleShadow( in highp vec4 shadowCoord ) {
    lowp float shadow = xll_shadow2D( _ShadowMapTexture, shadowCoord.xyz.xyz);
    shadow = (_LightShadowData.x + (shadow * (1.0 - _LightShadowData.x)));
    #line 388
    return shadow;
}
#line 453
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 455
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    surfIN.viewDir = IN.viewDir;
    #line 459
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    #line 463
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp float atten = unitySampleShadow( IN._ShadowCoord);
    #line 467
    lowp vec4 c = vec4( 0.0);
    lowp vec4 lmtex = texture( unity_Lightmap, IN.lmap.xy);
    lowp vec3 lm = DecodeLightmap( lmtex);
    c.xyz += (o.Albedo * min( lm, vec3( (atten * 2.0))));
    #line 471
    c.w = o.Alpha;
    c.xyz += o.Emission;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in highp vec3 xlv_TEXCOORD1;
in highp vec2 xlv_TEXCOORD2;
in highp vec4 xlv_TEXCOORD3;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD1);
    xlt_IN.lmap = vec2(xlv_TEXCOORD2);
    xlt_IN._ShadowCoord = vec4(xlv_TEXCOORD3);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "SHADOWS_NATIVE" "VERTEXLIGHT_ON" }
"!!GLES


#ifdef VERTEX

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_9;
  highp vec3 tmpvar_10;
  tmpvar_9 = tmpvar_1.xyz;
  tmpvar_10 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_11;
  tmpvar_11[0].x = tmpvar_9.x;
  tmpvar_11[0].y = tmpvar_10.x;
  tmpvar_11[0].z = tmpvar_2.x;
  tmpvar_11[1].x = tmpvar_9.y;
  tmpvar_11[1].y = tmpvar_10.y;
  tmpvar_11[1].z = tmpvar_2.y;
  tmpvar_11[2].x = tmpvar_9.z;
  tmpvar_11[2].y = tmpvar_10.z;
  tmpvar_11[2].z = tmpvar_2.z;
  highp vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_11 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_12;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_14;
  tmpvar_14.w = 1.0;
  tmpvar_14.xyz = tmpvar_8;
  mediump vec3 tmpvar_15;
  mediump vec4 normal_16;
  normal_16 = tmpvar_14;
  highp float vC_17;
  mediump vec3 x3_18;
  mediump vec3 x2_19;
  mediump vec3 x1_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAr, normal_16);
  x1_20.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAg, normal_16);
  x1_20.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHAb, normal_16);
  x1_20.z = tmpvar_23;
  mediump vec4 tmpvar_24;
  tmpvar_24 = (normal_16.xyzz * normal_16.yzzx);
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBr, tmpvar_24);
  x2_19.x = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBg, tmpvar_24);
  x2_19.y = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = dot (unity_SHBb, tmpvar_24);
  x2_19.z = tmpvar_27;
  mediump float tmpvar_28;
  tmpvar_28 = ((normal_16.x * normal_16.x) - (normal_16.y * normal_16.y));
  vC_17 = tmpvar_28;
  highp vec3 tmpvar_29;
  tmpvar_29 = (unity_SHC.xyz * vC_17);
  x3_18 = tmpvar_29;
  tmpvar_15 = ((x1_20 + x2_19) + x3_18);
  shlight_3 = tmpvar_15;
  tmpvar_6 = shlight_3;
  highp vec3 tmpvar_30;
  tmpvar_30 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_31;
  tmpvar_31 = (unity_4LightPosX0 - tmpvar_30.x);
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosY0 - tmpvar_30.y);
  highp vec4 tmpvar_33;
  tmpvar_33 = (unity_4LightPosZ0 - tmpvar_30.z);
  highp vec4 tmpvar_34;
  tmpvar_34 = (((tmpvar_31 * tmpvar_31) + (tmpvar_32 * tmpvar_32)) + (tmpvar_33 * tmpvar_33));
  highp vec4 tmpvar_35;
  tmpvar_35 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_31 * tmpvar_8.x) + (tmpvar_32 * tmpvar_8.y)) + (tmpvar_33 * tmpvar_8.z)) * inversesqrt(tmpvar_34))) * (1.0/((1.0 + (tmpvar_34 * unity_4LightAtten0)))));
  highp vec3 tmpvar_36;
  tmpvar_36 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_35.x) + (unity_LightColor[1].xyz * tmpvar_35.y)) + (unity_LightColor[2].xyz * tmpvar_35.z)) + (unity_LightColor[3].xyz * tmpvar_35.w)));
  tmpvar_6 = tmpvar_36;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform lowp sampler2DShadow _ShadowMapTexture;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 tmpvar_2;
  mediump float rim_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_5;
  tmpvar_5 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  highp float tmpvar_6;
  tmpvar_6 = (1.0 - clamp (dot (normalize(xlv_TEXCOORD1), tmpvar_5), 0.0, 1.0));
  rim_3 = tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_7 = (_RimColor.xyz * pow (rim_3, _RimPower));
  tmpvar_2 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_9;
  tmpvar_9 = tmpvar_8.w;
  lowp float shadow_10;
  lowp float tmpvar_11;
  tmpvar_11 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD4.xyz);
  highp float tmpvar_12;
  tmpvar_12 = (_LightShadowData.x + (tmpvar_11 * (1.0 - _LightShadowData.x)));
  shadow_10 = tmpvar_12;
  highp vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_14;
  lightDir_14 = xlv_TEXCOORD2;
  mediump vec3 viewDir_15;
  viewDir_15 = tmpvar_13;
  mediump float atten_16;
  atten_16 = shadow_10;
  mediump vec4 c_17;
  highp float nh_18;
  mediump float tmpvar_19;
  tmpvar_19 = max (0.0, dot (tmpvar_5, lightDir_14));
  mediump float tmpvar_20;
  tmpvar_20 = max (0.0, dot (tmpvar_5, normalize((lightDir_14 + viewDir_15))));
  nh_18 = tmpvar_20;
  mediump float tmpvar_21;
  tmpvar_21 = ((1.0 - dot (normalize(viewDir_15), tmpvar_5)) + dot (normalize(lightDir_14), tmpvar_5));
  highp vec3 tmpvar_22;
  tmpvar_22 = ((((tmpvar_4.xyz * _LightColor0.xyz) * tmpvar_19) + ((((_LightColor0.xyz * pow (nh_18, 48.0)) * tmpvar_8.w) * _Shininess) * _SpecColor.xyz)) * (atten_16 * 2.0));
  c_17.xyz = tmpvar_22;
  highp vec3 tmpvar_23;
  tmpvar_23 = (c_17.xyz + (c_17.xyz * (pow (tmpvar_21, _RimPower) * _RimColor.xyz)));
  c_17.xyz = tmpvar_23;
  c_17.w = tmpvar_9;
  c_1 = c_17;
  c_1.xyz = (c_1.xyz + (tmpvar_4.xyz * xlv_TEXCOORD3));
  c_1.xyz = (c_1.xyz + tmpvar_2);
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "SHADOWS_NATIVE" "VERTEXLIGHT_ON" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform lowp sampler2DShadow _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 433
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 95
highp vec3 Shade4PointLights( in highp vec4 lightPosX, in highp vec4 lightPosY, in highp vec4 lightPosZ, in highp vec3 lightColor0, in highp vec3 lightColor1, in highp vec3 lightColor2, in highp vec3 lightColor3, in highp vec4 lightAttenSq, in highp vec3 pos, in highp vec3 normal ) {
    highp vec4 toLightX = (lightPosX - pos.x);
    highp vec4 toLightY = (lightPosY - pos.y);
    #line 99
    highp vec4 toLightZ = (lightPosZ - pos.z);
    highp vec4 lengthSq = vec4( 0.0);
    lengthSq += (toLightX * toLightX);
    lengthSq += (toLightY * toLightY);
    #line 103
    lengthSq += (toLightZ * toLightZ);
    highp vec4 ndotl = vec4( 0.0);
    ndotl += (toLightX * normal.x);
    ndotl += (toLightY * normal.y);
    #line 107
    ndotl += (toLightZ * normal.z);
    highp vec4 corr = inversesqrt(lengthSq);
    ndotl = max( vec4( 0.0, 0.0, 0.0, 0.0), (ndotl * corr));
    highp vec4 atten = (1.0 / (1.0 + (lengthSq * lightAttenSq)));
    #line 111
    highp vec4 diff = (ndotl * atten);
    highp vec3 col = vec3( 0.0);
    col += (lightColor0 * diff.x);
    col += (lightColor1 * diff.y);
    #line 115
    col += (lightColor2 * diff.z);
    col += (lightColor3 * diff.w);
    return col;
}
#line 136
mediump vec3 ShadeSH9( in mediump vec4 normal ) {
    mediump vec3 x1;
    mediump vec3 x2;
    mediump vec3 x3;
    x1.x = dot( unity_SHAr, normal);
    #line 140
    x1.y = dot( unity_SHAg, normal);
    x1.z = dot( unity_SHAb, normal);
    mediump vec4 vB = (normal.xyzz * normal.yzzx);
    x2.x = dot( unity_SHBr, vB);
    #line 144
    x2.y = dot( unity_SHBg, vB);
    x2.z = dot( unity_SHBb, vB);
    highp float vC = ((normal.x * normal.x) - (normal.y * normal.y));
    x3 = (unity_SHC.xyz * vC);
    #line 148
    return ((x1 + x2) + x3);
}
#line 435
v2f_surf vert_surf( in appdata_full v ) {
    #line 437
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 441
    highp vec3 worldN = (mat3( _Object2World) * (v.normal * unity_Scale.w));
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    #line 445
    o.lightDir = lightDir;
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    highp vec3 shlight = ShadeSH9( vec4( worldN, 1.0));
    #line 449
    o.vlight = shlight;
    highp vec3 worldPos = (_Object2World * v.vertex).xyz;
    o.vlight += Shade4PointLights( unity_4LightPosX0, unity_4LightPosY0, unity_4LightPosZ0, unity_LightColor[0].xyz, unity_LightColor[1].xyz, unity_LightColor[2].xyz, unity_LightColor[3].xyz, unity_4LightAtten0, worldPos, worldN);
    o._ShadowCoord = (unity_World2Shadow[0] * (_Object2World * v.vertex));
    #line 454
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out highp vec3 xlv_TEXCOORD1;
out lowp vec3 xlv_TEXCOORD2;
out lowp vec3 xlv_TEXCOORD3;
out highp vec4 xlv_TEXCOORD4;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD2 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD3 = vec3(xl_retval.vlight);
    xlv_TEXCOORD4 = vec4(xl_retval._ShadowCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_shadow2D(mediump sampler2DShadow s, vec3 coord) { return texture (s, coord); }
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 408
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 423
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    highp vec3 viewDir;
    lowp vec3 lightDir;
    lowp vec3 vlight;
    highp vec4 _ShadowCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp vec4 _ShadowOffsets[4];
uniform lowp sampler2DShadow _ShadowMapTexture;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 392
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 415
#line 433
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 395
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    #line 397
    mediump vec3 h = normalize((lightDir + viewDir));
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    #line 401
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    #line 405
    c.w = s.Alpha;
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 415
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 419
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 384
lowp float unitySampleShadow( in highp vec4 shadowCoord ) {
    lowp float shadow = xll_shadow2D( _ShadowMapTexture, shadowCoord.xyz.xyz);
    shadow = (_LightShadowData.x + (shadow * (1.0 - _LightShadowData.x)));
    #line 388
    return shadow;
}
#line 456
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 458
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    surfIN.viewDir = IN.viewDir;
    #line 462
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    #line 466
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp float atten = unitySampleShadow( IN._ShadowCoord);
    #line 470
    lowp vec4 c = vec4( 0.0);
    c = LightingSimpleSpecular( o, IN.lightDir, normalize(IN.viewDir), atten);
    c.xyz += (o.Albedo * IN.vlight);
    c.xyz += o.Emission;
    #line 474
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in highp vec3 xlv_TEXCOORD1;
in lowp vec3 xlv_TEXCOORD2;
in lowp vec3 xlv_TEXCOORD3;
in highp vec4 xlv_TEXCOORD4;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD1);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD2);
    xlt_IN.vlight = vec3(xlv_TEXCOORD3);
    xlt_IN._ShadowCoord = vec4(xlv_TEXCOORD4);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

}
Program "fp" {
// Fragment combos: 4
//   opengl - ALU: 20 to 52, TEX: 2 to 4
//   d3d9 - ALU: 24 to 62, TEX: 2 to 4
//   d3d11 - ALU: 15 to 40, TEX: 2 to 4, FLOW: 1 to 1
//   d3d11_9x - ALU: 15 to 40, TEX: 2 to 4, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 50 ALU, 2 TEX
PARAM c[6] = { program.local[0..4],
		{ 2, 1, 0, 48 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R1.yw, fragment.texcoord[0].zwzw, texture[1], 2D;
DP3 R1.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R1.z, R1.x;
MUL R2.xyz, R1.z, fragment.texcoord[1];
MAD R1.xy, R1.wyzw, c[5].x, -c[5].y;
MUL R1.zw, R1.xyxy, R1.xyxy;
ADD_SAT R1.z, R1, R1.w;
ADD R3.xyz, fragment.texcoord[2], R2;
DP3 R1.w, R3, R3;
RSQ R1.w, R1.w;
ADD R1.z, -R1, c[5].y;
RSQ R1.z, R1.z;
DP3 R2.w, fragment.texcoord[2], fragment.texcoord[2];
RCP R1.z, R1.z;
MUL R3.xyz, R1.w, R3;
DP3 R1.w, R1, R3;
MAX R1.w, R1, c[5].z;
POW R1.w, R1.w, c[5].w;
MUL R3.xyz, R1.w, c[0];
DP3 R1.w, R1, fragment.texcoord[2];
MUL R3.xyz, R3, R0.w;
MAX R1.w, R1, c[5].z;
MUL R4.xyz, R0, c[0];
MUL R4.xyz, R4, R1.w;
DP3 R1.w, R2, R2;
RSQ R1.w, R1.w;
MUL R2.xyz, R1.w, R2;
MUL R3.xyz, R3, c[2].x;
MAD R3.xyz, R3, c[1], R4;
DP3 R1.w, R1, R2;
RSQ R2.w, R2.w;
MUL R2.xyz, R2.w, fragment.texcoord[2];
DP3 R2.x, R1, R2;
ADD R1.w, -R1, R2.x;
DP3 R2.w, fragment.texcoord[1], fragment.texcoord[1];
RSQ R2.y, R2.w;
MUL R2.xyz, R2.y, fragment.texcoord[1];
DP3_SAT R1.y, R1, R2;
ADD R1.w, R1, c[5].y;
POW R1.x, R1.w, c[4].x;
ADD R1.w, -R1.y, c[5].y;
POW R1.w, R1.w, c[4].x;
MUL R3.xyz, R3, c[5].x;
MUL R1.xyz, R1.x, c[3];
MAD R1.xyz, R3, R1, R3;
MUL R2.xyz, R1.w, c[3];
MAD R0.xyz, fragment.texcoord[3], R0, R1;
ADD result.color.xyz, R0, R2;
MOV result.color.w, R0;
END
# 50 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
"ps_2_0
; 61 ALU, 2 TEX
dcl_2d s0
dcl_2d s1
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 48.00000000, 0, 0, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
texld r2, t0, s0
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s1
mov r1.x, r0.w
mov r1.y, r0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r4.xyz, r0.x, t1
mad_pp r3.xy, r1, c5.x, c5.y
add_pp r5.xyz, t2, r4
mul_pp r0.xy, r3, r3
add_pp_sat r0.x, r0, r0.y
dp3_pp r1.x, r5, r5
add_pp r0.x, -r0, c5.z
rsq_pp r0.x, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r5
rcp_pp r3.z, r0.x
dp3_pp r0.x, r3, r1
max_pp r0.x, r0, c5.w
pow r1.w, r0.x, c6.x
mov r0.x, r1.w
mul r1.xyz, r0.x, c0
dp3_pp r0.x, r3, t2
mul r1.xyz, r1, r2.w
mul r1.xyz, r1, c2.x
mul_pp r5.xyz, r2, c0
max_pp r0.x, r0, c5.w
mul_pp r0.xyz, r5, r0.x
mad r0.xyz, r1, c1, r0
mul r5.xyz, r0, c5.x
dp3_pp r1.x, r4, r4
dp3_pp r0.x, t2, t2
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r4
rsq_pp r0.x, r0.x
mul_pp r4.xyz, r0.x, t2
dp3_pp r0.x, r3, r1
dp3_pp r4.x, r3, r4
add_pp r0.x, -r0, r4
dp3 r1.x, t1, t1
rsq r1.x, r1.x
add_pp r0.x, r0, c5.z
mul r4.xyz, r1.x, t1
pow_pp r1.w, r0.x, c4.x
dp3_sat r0.x, r3, r4
add r0.x, -r0, c5.z
pow_pp r3.x, r0.x, c4.x
mul r0.xyz, r1.w, c3
mad r1.xyz, r5, r0, r5
mov_pp r0.x, r3.x
mad_pp r1.xyz, t3, r2, r1
mul r0.xyz, r0.x, c3
mov_pp r0.w, r2
add_pp r0.xyz, r1, r0
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 3 [_RimColor]
Float 4 [_RimPower]
Float 2 [_Shininess]
Vector 1 [_SpecColor]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 42.67 (32 instructions), vertex: 0, texture: 8,
//   sequencer: 16, interpolator: 16;    9 GPRs, 21 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabmaaaaaacbeaaaaaaaaaaaaaaceaaaaabgiaaaaabjaaaaaaaaa
aaaaaaaaaaaaabeaaaaaaabmaaaaabdbppppadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabckaaaaaakiaaadaaabaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaaa
aaabaaaaaaaaaaneaaaaaaaaaaaaaaoeaaadaaaaaaabaaaaaaaaaaleaaaaaaaa
aaaaaaonaaacaaadaaabaaaaaaaaaaneaaaaaaaaaaaaaaphaaacaaaeaaabaaaa
aaaaabaeaaaaaaaaaaaaabbeaaacaaacaaabaaaaaaaaabaeaaaaaaaaaaaaabbp
aaacaaabaaabaaaaaaaaaaneaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaam
aaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcdaaaklklklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhiaafpfcgjgnedgpgmgphcaafp
fcgjgnfagphhgfhcaaklklklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpfdgigj
gogjgogfhdhdaafpfdhagfgdedgpgmgphcaahahdfpddfpdaaadccodacodcdadd
dfddcodaaaklklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaaba
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabnebaaaaiaaaaaaaaae
aaaaaaaaaaaadeieaaapaaapaaaaaaabaaaapafaaaaahbfbaaaahcfcaaaahdfd
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
dpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaaaaaaaaaalpiaaaaaeceaaaaa
aaafcaaeaaaabcaameaaaaaaaaaagaaggaambcaabcaaaaaaaaaagabcgabibcaa
bcaaaaaaaaaagabocacebcaaccaaaaaadibifaabbpbpppnjaaaaeaaabaaieaab
bpbppgiiaaaaeaaamiacaaaaaaloloaapaacacaabeiaiaaaaaaaaablocaaaaae
miadaaafaagngmmgilafppppficbaaaaaalololbpaababiafibhaaaiaalbmagm
obaaaciamiahaaagaagmmaaaobaaabaamjacaaaaaalalalbnbafafppmiaoaaab
aapmpmaaoaagacaalibeaaaaaaloloebnaagagpoficiaaaaaamdmdmgpaababia
kaehafahaamalbgmobagaaiamiacaaaaaaloloaapaafacaamiabaaabaaloloaa
paaiafaamiabaaaaaaloloaapaahafaafjieaaaaaaloloblpaagafiamiaoaaab
aaabblaaobabaaaamiaiaaaaaamdloaapaabafaamiafaaaaaemegmaakaaapoaa
eaebaaaaaagmgmmgoaaaabiaeacfaaacaaldlbgmkcaappiaeabaaaaaaaaaaagm
ocaaaaickibhaaafaaleleaaibaeaappdibdaaacaamfgmgmkbaaaeaadiihaaaa
aagmmagmkbaaaaackibhabaaaamabledmbaaaeadkichabaaaalegmmdibaaacad
kiehabaaaamaleidibaaabadmiahaaaaaamamgmaolafacaadiihaaaaaalelelb
oaaaaaacmiahaaaaaamaleleolabaaaamiahaaaaaamamaleolaeadaamiahiaaa
aablmamaklaaadaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
"sce_fp_rsx // 59 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840004000000
[Offsets]
5
_LightColor0 2 0
00000230000001c0
_SpecColor 1 0
00000270
_Shininess 1 0
00000200
_RimColor 2 0
0000039000000310
_RimPower 2 0
00000350000002a0
[Microcode]
944
940017025c011c9dc8000001c8003fe106800440ce001c9d00020000aa020000
000040000000bf800000000000000000ae843940c8011c9dc8000029c800bfe1
0880b840c9001c9dc9000001c8000001ce860140c8011c9dc8000001c8003fe1
1084034055001c9f00020000c800000100003f80000000000000000000000000
0e040340c90c1c9dc9080001c800000108803b40ff083c9dff080001c8000001
10800540c9001c9dc90c0001c80000010e843940c9081c9dc8000029c8000001
10820540c9001c9dc9080001c800000110800900c9001c9d00020000c8000001
000000000000000000000000000000009e021700c8011c9dc8000001c8003fe1
0e883940c8081c9dc8000029c800000102880540c9001c9dc9100001c8000001
ce823940c8011c9dc8000029c8003fe104880540c9001c9dc9040001c8000001
1004090001101c9c00020000c800000100000000000000000000000000000000
028a0340ff041c9fab100000c800000108001d00fe081c9dc8000001c8000001
0204020054001c9d00020000c800000100004240000000000000000000000000
02041c00c8081c9dc8000001c80000010e820240c8041c9dc8020001c8000001
0000000000000000000000000000000010040200c8041c9d00080000c8000001
a2040500c8011c9dc8010001c800bfe110040200c8081c9d00020000c8000001
000000000000000000000000000000000e820240c9041c9dff000001c8000001
0e060200fe081c9dc8020001c800000100000000000000000000000000000000
108e034001141c9cc8020001c800000100000000000000000000000000003f80
0e060400c80c1c9dc8021001c904000100000000000000000000000000000000
08001d00ff1c1c9dc8000001c80000011004020054001c9d00020000c8000001
00000000000000000000000000000000ae043b00c8011c9dc8080001c800bfe1
02008500c8081c9dc9000001c800000104001c00fe081c9dc8000001c8000001
1000030000001c9ec8020001c800000100000000000000000000000000003f80
0e000200aa001c9cc8020001c800000100000000000000000000000000000000
0e800400c80c1c9dc8000001c80c000108001d00fe001c9dc8000001c8000001
1000020054001c9d00020000c800000100000000000000000000000000000000
ee800440c8041c9dc8010001c9003fe110801c00fe001c9dc8000001c8000001
0e800440ff001c9dc8020001c900000100000000000000000000000000000000
10810140c8041c9dc8000001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 128 // 84 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 48 [_Shininess]
Vector 64 [_RimColor] 4
Float 80 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_BumpMap] 2D 1
// 49 instructions, 5 temp regs, 0 temp arrays:
// ALU 38 float, 0 int, 0 uint
// TEX 2 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedcnljjeppfhpkkmpgklngegiaonkmbmnoabaaaaaaaeahaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcpmafaaaaeaaaaaaahpabaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafibiaaaeaahabaaa
aaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaagcbaaaadpcbabaaa
abaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaad
hcbabaaaaeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaabaaaaaah
bcaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaaagbjbaaaacaaaaaa
agaabaaaaaaaaaaaagbjbaaaadaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaa
aaaaaaaaegbcbaaaacaaaaaabaaaaaahbcaabaaaaaaaaaaajgahbaaaaaaaaaaa
jgahbaaaaaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaaefaaaaajpcaabaaa
acaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaadcaaaaap
dcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaa
aaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaaapaaaaahicaabaaa
aaaaaaaaegaabaaaacaaaaaaegaabaaaacaaaaaaddaaaaahicaabaaaaaaaaaaa
dkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaaaaaaaaaadkaabaia
ebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaaacaaaaaadkaabaaa
aaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaaaaaaaaa
deaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaaaaacpaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaabeaaaaaaaaaeaecbjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaaihcaabaaaaaaaaaaaagaabaaaaaaaaaaaegiccaaaaaaaaaaaabaaaaaa
efaaaaajpcaabaaaadaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaadaaaaaa
diaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaagiacaaaaaaaaaaaadaaaaaa
diaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaacaaaaaa
baaaaaahicaabaaaaaaaaaaaegacbaaaacaaaaaaegbcbaaaadaaaaaadeaaaaah
icaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaadiaaaaaihcaabaaa
aeaaaaaaegacbaaaadaaaaaaegiccaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaa
aaaaaaaaegacbaaaaeaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaaaaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaabaaaaaahicaabaaa
aaaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaaibcaabaaaabaaaaaa
dkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpdgcaaaaficaabaaaaaaaaaaa
dkaabaaaaaaaaaaaaaaaaaaiicaabaaaaaaaaaaadkaabaiaebaaaaaaaaaaaaaa
abeaaaaaaaaaiadpcpaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaai
icaabaaaaaaaaaaadkaabaaaaaaaaaaaakiacaaaaaaaaaaaafaaaaaabjaaaaaf
icaabaaaaaaaaaaadkaabaaaaaaaaaaabaaaaaahccaabaaaabaaaaaaegbcbaaa
adaaaaaaegbcbaaaadaaaaaaeeaaaaafccaabaaaabaaaaaabkaabaaaabaaaaaa
diaaaaahocaabaaaabaaaaaafgafbaaaabaaaaaaagbjbaaaadaaaaaabaaaaaah
ccaabaaaabaaaaaajgahbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaahbcaabaaa
abaaaaaabkaabaaaabaaaaaaakaabaaaabaaaaaacpaaaaafbcaabaaaabaaaaaa
akaabaaaabaaaaaadiaaaaaibcaabaaaabaaaaaaakaabaaaabaaaaaaakiacaaa
aaaaaaaaafaaaaaabjaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaagaabaaaabaaaaaaegiccaaaaaaaaaaaaeaaaaaadcaaaaaj
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaa
dcaaaaajhcaabaaaaaaaaaaaegacbaaaadaaaaaaegbcbaaaaeaaaaaaegacbaaa
aaaaaaaadgaaaaaficcabaaaaaaaaaaadkaabaaaadaaaaaadcaaaaakhccabaaa
aaaaaaaaegiccaaaaaaaaaaaaeaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
doaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
"agal_ps
c5 2.0 -1.0 1.0 0.0
c6 48.0 0.0 0.0 0.0
[bc]
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
ciaaaaaaaaaaapacaaaaaafeacaaaaaaabaaaaaaafaababb tex r0, r0.xyyy, s1 <2d wrap linear point>
aaaaaaaaabaaabacaaaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r0.w
aaaaaaaaabaaacacaaaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r1.y, r0.y
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaaeaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r4.xyz, r0.x, v1
adaaaaaaadaaadacabaaaafeacaaaaaaafaaaaaaabaaaaaa mul r3.xy, r1.xyyy, c5.x
abaaaaaaadaaadacadaaaafeacaaaaaaafaaaaffabaaaaaa add r3.xy, r3.xyyy, c5.y
abaaaaaaafaaahacacaaaaoeaeaaaaaaaeaaaakeacaaaaaa add r5.xyz, v2, r4.xyzz
adaaaaaaaaaaabacadaaaaffacaaaaaaadaaaaffacaaaaaa mul r0.x, r3.y, r3.y
bfaaaaaaabaaaiacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.w, r3.x
adaaaaaaabaaaiacabaaaappacaaaaaaadaaaaaaacaaaaaa mul r1.w, r1.w, r3.x
acaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.w, r0.x
bcaaaaaaabaaabacafaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r5.xyzz
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaafaaaakeacaaaaaa mul r1.xyz, r1.x, r5.xyzz
afaaaaaaadaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r3.z, r0.x
bcaaaaaaaaaaabacadaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r3.xyzz, r1.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
alaaaaaaabaaapacaaaaaaaaacaaaaaaagaaaaaaabaaaaaa pow r1, r0.x, c6.x
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
adaaaaaaabaaahacaaaaaaaaacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.x, c0
bcaaaaaaaaaaabacadaaaakeacaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, r3.xyzz, v2
adaaaaaaabaaahacabaaaakeacaaaaaaacaaaappacaaaaaa mul r1.xyz, r1.xyzz, r2.w
adaaaaaaabaaahacabaaaakeacaaaaaaacaaaaaaabaaaaaa mul r1.xyz, r1.xyzz, c2.x
adaaaaaaafaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r5.xyz, r2.xyzz, c0
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
adaaaaaaaaaaahacafaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r5.xyzz, r0.x
adaaaaaaagaaahacabaaaakeacaaaaaaabaaaaoeabaaaaaa mul r6.xyz, r1.xyzz, c1
abaaaaaaaaaaahacagaaaakeacaaaaaaaaaaaakeacaaaaaa add r0.xyz, r6.xyzz, r0.xyzz
adaaaaaaafaaahacaaaaaakeacaaaaaaafaaaaaaabaaaaaa mul r5.xyz, r0.xyzz, c5.x
bcaaaaaaabaaabacaeaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r4.xyzz
bcaaaaaaaaaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, v2, v2
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaaeaaaakeacaaaaaa mul r1.xyz, r1.x, r4.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaaeaaahacaaaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r4.xyz, r0.x, v2
bcaaaaaaaaaaabacadaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r3.xyzz, r1.xyzz
bcaaaaaaaeaaabacadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r4.x, r3.xyzz, r4.xyzz
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaaaaacaaaaaa add r0.x, r0.x, r4.x
bcaaaaaaabaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, v1, v1
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
adaaaaaaaeaaahacabaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r4.xyz, r1.x, v1
alaaaaaaabaaapacaaaaaaaaacaaaaaaaeaaaaaaabaaaaaa pow r1, r0.x, c4.x
bcaaaaaaaaaaabacadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r0.x, r3.xyzz, r4.xyzz
bgaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa sat r0.x, r0.x
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
alaaaaaaadaaapacaaaaaaaaacaaaaaaaeaaaaaaabaaaaaa pow r3, r0.x, c4.x
adaaaaaaaaaaahacabaaaaaaacaaaaaaadaaaaoeabaaaaaa mul r0.xyz, r1.x, c3
adaaaaaaabaaahacafaaaakeacaaaaaaaaaaaakeacaaaaaa mul r1.xyz, r5.xyzz, r0.xyzz
abaaaaaaabaaahacabaaaakeacaaaaaaafaaaakeacaaaaaa add r1.xyz, r1.xyzz, r5.xyzz
aaaaaaaaaaaaabacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r3.x
adaaaaaaagaaahacadaaaaoeaeaaaaaaacaaaakeacaaaaaa mul r6.xyz, v3, r2.xyzz
abaaaaaaabaaahacagaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r6.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaaaaacaaaaaaadaaaaoeabaaaaaa mul r0.xyz, r0.x, c3
aaaaaaaaaaaaaiacacaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.w, r2.w
abaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa add r0.xyz, r1.xyzz, r0.xyzz
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 128 // 84 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 48 [_Shininess]
Vector 64 [_RimColor] 4
Float 80 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_BumpMap] 2D 1
// 49 instructions, 5 temp regs, 0 temp arrays:
// ALU 38 float, 0 int, 0 uint
// TEX 2 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_1
eefiecedojidjbhccomfpjnbgklaiinjidighhjpabaaaaaagaakaaaaaeaaaaaa
daaaaaaaiiadaaaaimajaaaacmakaaaaebgpgodjfaadaaaafaadaaaaaaacpppp
biadaaaadiaaaaaaabaacmaaaaaadiaaaaaadiaaacaaceaaaaaadiaaaaaaaaaa
abababaaaaaaabaaafaaaaaaaaaaaaaaaaacppppfbaaaaafafaaapkaaaaaaaea
aaaaialpaaaaaaaaaaaaiadpfbaaaaafagaaapkaaaaaeaecaaaaaaaaaaaaaaaa
aaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaac
aaaaaaiaacaachlabpaaaaacaaaaaaiaadaachlabpaaaaacaaaaaajaaaaiapka
bpaaaaacaaaaaajaabaiapkaaiaaaaadaaaaaiiaabaaoelaabaaoelaahaaaaac
aaaaabiaaaaappiaabaaaaacabaaahiaabaaoelaaeaaaaaeabaachiaabaaoeia
aaaaaaiaacaaoelaafaaaaadaaaaahiaaaaaaaiaabaaoelaceaaaaacacaachia
abaaoeiaabaaaaacabaaabiaaaaakklaabaaaaacabaaaciaaaaapplaecaaaaad
abaacpiaabaaoeiaabaioekaecaaaaadadaaapiaaaaaoelaaaaioekaaeaaaaae
aeaacbiaabaappiaafaaaakaafaaffkaaeaaaaaeaeaacciaabaaffiaafaaaaka
afaaffkafkaaaaaeaaaadiiaaeaaoeiaaeaaoeiaafaakkkaacaaaaadaaaaciia
aaaappibafaappkaahaaaaacaaaaciiaaaaappiaagaaaaacaeaaceiaaaaappia
aiaaaaadaaaaciiaaeaaoeiaacaaoeiaalaaaaadaeaaaiiaaaaappiaafaakkka
caaaaaadaaaaaiiaaeaappiaagaaaakaafaaaaadabaaahiaaaaappiaaaaaoeka
afaaaaadabaaahiaadaappiaabaaoeiaafaaaaadabaaahiaabaaoeiaacaaaaka
afaaaaadabaaahiaabaaoeiaabaaoekaaiaaaaadaaaaciiaaeaaoeiaacaaoela
alaaaaadabaaciiaaaaappiaafaakkkaafaaaaadacaachiaadaaoeiaaaaaoeka
aeaaaaaeabaaahiaacaaoeiaabaappiaabaaoeiaacaaaaadabaachiaabaaoeia
abaaoeiaaiaaaaadabaaaiiaaaaaoeiaaeaaoeiaacaaaaadaeaaciiaabaappib
afaappkaabaaaaacabaabiiaabaappiaacaaaaadabaaciiaabaappibafaappka
caaaaaadaaaaabiaabaappiaaeaaaakaceaaaaacacaachiaacaaoelaaiaaaaad
abaaciiaacaaoeiaaeaaoeiaacaaaaadabaaciiaabaappiaaeaappiacaaaaaad
aaaaaciaabaappiaaeaaaakaafaaaaadaaaaaoiaaaaaffiaadaablkaaeaaaaae
aaaacoiaabaabliaaaaaoeiaabaabliaaeaaaaaeaaaacoiaadaabliaadaablla
aaaaoeiaaeaaaaaeadaachiaadaaoekaaaaaaaiaaaaabliaabaaaaacaaaicpia
adaaoeiappppaaaafdeieefcpmafaaaaeaaaaaaahpabaaaafjaaaaaeegiocaaa
aaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
gcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaa
adaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaac
afaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaa
eeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaa
agbjbaaaacaaaaaaagaabaaaaaaaaaaaagbjbaaaadaaaaaadiaaaaahhcaabaaa
abaaaaaaagaabaaaaaaaaaaaegbcbaaaacaaaaaabaaaaaahbcaabaaaaaaaaaaa
jgahbaaaaaaaaaaajgahbaaaaaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaa
efaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
abaaaaaadcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
apaaaaahicaabaaaaaaaaaaaegaabaaaacaaaaaaegaabaaaacaaaaaaddaaaaah
icaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaa
aaaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaa
acaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaa
egacbaaaaaaaaaaadeaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaeaecbjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadiaaaaaihcaabaaaaaaaaaaaagaabaaaaaaaaaaaegiccaaa
aaaaaaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaegbabaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaa
pgapbaaaadaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaagiacaaa
aaaaaaaaadaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaa
aaaaaaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaacaaaaaaegbcbaaa
adaaaaaadeaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaa
diaaaaaihcaabaaaaeaaaaaaegacbaaaadaaaaaaegiccaaaaaaaaaaaabaaaaaa
dcaaaaajhcaabaaaaaaaaaaaegacbaaaaeaaaaaapgapbaaaaaaaaaaaegacbaaa
aaaaaaaaaaaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaa
baaaaaahicaabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaai
bcaabaaaabaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpdgcaaaaf
icaabaaaaaaaaaaadkaabaaaaaaaaaaaaaaaaaaiicaabaaaaaaaaaaadkaabaia
ebaaaaaaaaaaaaaaabeaaaaaaaaaiadpcpaaaaaficaabaaaaaaaaaaadkaabaaa
aaaaaaaadiaaaaaiicaabaaaaaaaaaaadkaabaaaaaaaaaaaakiacaaaaaaaaaaa
afaaaaaabjaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaabaaaaaahccaabaaa
abaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafccaabaaaabaaaaaa
bkaabaaaabaaaaaadiaaaaahocaabaaaabaaaaaafgafbaaaabaaaaaaagbjbaaa
adaaaaaabaaaaaahccaabaaaabaaaaaajgahbaaaabaaaaaaegacbaaaacaaaaaa
aaaaaaahbcaabaaaabaaaaaabkaabaaaabaaaaaaakaabaaaabaaaaaacpaaaaaf
bcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaaibcaabaaaabaaaaaaakaabaaa
abaaaaaaakiacaaaaaaaaaaaafaaaaaabjaaaaafbcaabaaaabaaaaaaakaabaaa
abaaaaaadiaaaaaihcaabaaaabaaaaaaagaabaaaabaaaaaaegiccaaaaaaaaaaa
aeaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaa
egacbaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaadaaaaaaegbcbaaa
aeaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaadkaabaaaadaaaaaa
dcaaaaakhccabaaaaaaaaaaaegiccaaaaaaaaaaaaeaaaaaapgapbaaaaaaaaaaa
egacbaaaaaaaaaaadoaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES3"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [unity_Lightmap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 20 ALU, 3 TEX
PARAM c[3] = { program.local[0..1],
		{ 8, 2, 1 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R1, fragment.texcoord[0], texture[0], 2D;
TEX R2.yw, fragment.texcoord[0].zwzw, texture[1], 2D;
TEX R0, fragment.texcoord[2], texture[2], 2D;
MAD R2.xy, R2.wyzw, c[2].y, -c[2].z;
MUL R2.zw, R2.xyxy, R2.xyxy;
ADD_SAT R2.z, R2, R2.w;
ADD R2.w, -R2.z, c[2].z;
MUL R0.xyz, R0.w, R0;
RSQ R3.x, R2.w;
DP3 R2.z, fragment.texcoord[1], fragment.texcoord[1];
RSQ R2.w, R2.z;
RCP R2.z, R3.x;
MUL R3.xyz, R2.w, fragment.texcoord[1];
DP3_SAT R2.x, R3, R2;
ADD R2.x, -R2, c[2].z;
POW R2.x, R2.x, c[1].x;
MUL R2.xyz, R2.x, c[0];
MUL R0.xyz, R0, R1;
MAD result.color.xyz, R0, c[2].x, R2;
MOV result.color.w, R1;
END
# 20 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [unity_Lightmap] 2D
"ps_2_0
; 24 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
def c2, 2.00000000, -1.00000000, 1.00000000, 8.00000000
dcl t0
dcl t1.xyz
dcl t2.xy
texld r2, t0, s0
texld r3, t2, s2
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s1
mov r0.x, r0.w
mad_pp r4.xy, r0, c2.x, c2.y
mul_pp r0.xy, r4, r4
add_pp_sat r0.x, r0, r0.y
add_pp r1.x, -r0, c2.z
rsq_pp r1.x, r1.x
dp3 r0.x, t1, t1
rsq r0.x, r0.x
mul r0.xyz, r0.x, t1
rcp_pp r4.z, r1.x
dp3_sat r0.x, r0, r4
add r0.x, -r0, c2.z
pow_pp r1.w, r0.x, c1.x
mov_pp r0.x, r1.w
mul_pp r1.xyz, r3.w, r3
mul_pp r1.xyz, r1, r2
mul r0.xyz, r0.x, c0
mov_pp r0.w, r2
mad_pp r0.xyz, r1, c2.w, r0
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [unity_Lightmap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 17.33 (13 instructions), vertex: 0, texture: 12,
//   sequencer: 10, interpolator: 12;    5 GPRs, 36 threads,
// Performance (if enough threads): ~17 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabhmaaaaabdaaaaaaaaaaaaaaaceaaaaabciaaaaabfaaaaaaaaa
aaaaaaaaaaaaabaaaaaaaabmaaaaaapcppppadaaaaaaaaafaaaaaabmaaaaaaaa
aaaaaaolaaaaaaiaaaadaaabaaabaaaaaaaaaaimaaaaaaaaaaaaaajmaaadaaaa
aaabaaaaaaaaaaimaaaaaaaaaaaaaakfaaacaaaaaaabaaaaaaaaaalaaaaaaaaa
aaaaaamaaaacaaabaaabaaaaaaaaaammaaaaaaaaaaaaaanmaaadaaacaaabaaaa
aaaaaaimaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaa
aaaaaaaafpengbgjgofegfhiaafpfcgjgnedgpgmgphcaaklaaabaaadaaabaaae
aaabaaaaaaaaaaaafpfcgjgnfagphhgfhcaaklklaaaaaaadaaabaaabaaabaaaa
aaaaaaaahfgogjhehjfpemgjghgihegngbhaaahahdfpddfpdaaadccodacodcda
dddfddcodaaaklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaaba
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaapabaaaaeaaaaaaaaae
aaaaaaaaaaaacegdaaahaaahaaaaaaabaaaapafaaaaahbfbaaaadcfcaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaa
aaaaaaaalpiaaaaadpiaaaaaebaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabfdaad
aaaabcaameaaaaaaaaaagaaggaambcaabcaaaaaaaaaababcaaaaccaaaaaaaaaa
bacidaebbpbppgiiaaaaeaaadibieaabbpbpppnjaaaaeaaabaaicaabbpbppgii
aaaaeaaamiaiaaaaaaloloaapaababaabeiaiaaaaaaaaablocaaaaacmiadaaaa
aagngmmgilaepopomjaeaaaaaalalalbnbaaaapofiieaaaaaemgblblkaaapoia
kaehaaabaablmamgobaaabiamjabaaaaaaloloaapaabaaaalibaaaaaaaaaaaaa
mcaaaapoeaceaaaaaablgmgmkbadppiakibaaaaaaaaaaaebmcaaaaabdibhaaab
aamgmagmobaaadaamiahaaaaaagmmaaakbaaaaaamiahiaaaaamamamaolabacaa
aaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [unity_Lightmap] 2D
"sce_fp_rsx // 23 instructions using 2 registers
[Configuration]
24
ffffffff0001c0200007fffc000000000000840002000000
[Offsets]
2
_RimColor 1 0
00000150
_RimPower 1 0
00000100
[Microcode]
368
940017025c011c9dc8000001c8003fe106800440ce001c9d00020000aa020000
000040000000bf800000000000000000a8000500c8011c9dc8010001c800bfe1
0880b840c9001c9dc9000001c80000011080034055001c9f00020000c8000001
00003f80000000000000000000000000ae023b00c8011c9d54000001c800bfe1
08803b40ff003c9dff000001c800000102008500c8041c9dc9000001c8000001
02000300c8001c9f00020000c800000100003f80000000000000000000000000
02021d00c8001c9dc8000001c8000001de001704c8011c9dc8000001c8003fe1
0e800240fe001c9dc8000001c80000011000020000041c9c00020000c8000001
000000000000000000000000000000009e021700c8011c9dc8000001c8003fe1
0e800240c9001c9dc8043001c800000110801c00fe001c9dc8000001c8000001
0e800440ff001c9dc8020001c900000100000000000000000000000000000000
10810140c8041c9dc8000001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 144 // 84 used size, 9 vars
Vector 64 [_RimColor] 4
Float 80 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_BumpMap] 2D 1
SetTexture 2 [unity_Lightmap] 2D 2
// 22 instructions, 3 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedlkfkbjldhijhjoldojdlchjmdoobffdcabaaaaaaamaeaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaadadaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcbmadaaaaeaaaaaaamhaaaaaafjaaaaaeegiocaaa
aaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaad
pcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaaddcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegbcbaaaacaaaaaa
efaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
abaaaaaadcaaaaapdcaabaaaabaaaaaahgapbaaaabaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
apaaaaahicaabaaaaaaaaaaaegaabaaaabaaaaaaegaabaaaabaaaaaaddaaaaah
icaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaa
aaaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaa
abaaaaaadkaabaaaaaaaaaaabacaaaahbcaabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaa
abeaaaaaaaaaiadpcpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaai
bcaabaaaaaaaaaaaakaabaaaaaaaaaaaakiacaaaaaaaaaaaafaaaaaabjaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaihcaabaaaaaaaaaaaagaabaaa
aaaaaaaaegiccaaaaaaaaaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaa
adaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaadiaaaaahicaabaaaaaaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaebdiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaaaaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaa
acaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaa
dkaabaaaacaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [unity_Lightmap] 2D
"agal_ps
c2 2.0 -1.0 1.0 8.0
[bc]
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
ciaaaaaaadaaapacacaaaaoeaeaaaaaaacaaaaaaafaababb tex r3, v2, s2 <2d wrap linear point>
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
ciaaaaaaaaaaapacaaaaaafeacaaaaaaabaaaaaaafaababb tex r0, r0.xyyy, s1 <2d wrap linear point>
aaaaaaaaaaaaabacaaaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r0.w
adaaaaaaaeaaadacaaaaaafeacaaaaaaacaaaaaaabaaaaaa mul r4.xy, r0.xyyy, c2.x
abaaaaaaaeaaadacaeaaaafeacaaaaaaacaaaaffabaaaaaa add r4.xy, r4.xyyy, c2.y
adaaaaaaaaaaabacaeaaaaffacaaaaaaaeaaaaffacaaaaaa mul r0.x, r4.y, r4.y
bfaaaaaaabaaabacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.x, r4.x
adaaaaaaabaaabacabaaaaaaacaaaaaaaeaaaaaaacaaaaaa mul r1.x, r1.x, r4.x
acaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.x, r0.x
abaaaaaaabaaabacaaaaaaaaacaaaaaaacaaaakkabaaaaaa add r1.x, r0.x, c2.z
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r0.xyz, r0.x, v1
afaaaaaaaeaaaeacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r4.z, r1.x
bcaaaaaaaaaaabacaaaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r0.x, r0.xyzz, r4.xyzz
bgaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa sat r0.x, r0.x
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaacaaaakkabaaaaaa add r0.x, r0.x, c2.z
alaaaaaaabaaapacaaaaaaaaacaaaaaaabaaaaaaabaaaaaa pow r1, r0.x, c1.x
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
adaaaaaaabaaahacadaaaappacaaaaaaadaaaakeacaaaaaa mul r1.xyz, r3.w, r3.xyzz
adaaaaaaabaaahacabaaaakeacaaaaaaacaaaakeacaaaaaa mul r1.xyz, r1.xyzz, r2.xyzz
adaaaaaaaaaaahacaaaaaaaaacaaaaaaaaaaaaoeabaaaaaa mul r0.xyz, r0.x, c0
aaaaaaaaaaaaaiacacaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.w, r2.w
adaaaaaaabaaahacabaaaakeacaaaaaaacaaaappabaaaaaa mul r1.xyz, r1.xyzz, c2.w
abaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa add r0.xyz, r1.xyzz, r0.xyzz
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 144 // 84 used size, 9 vars
Vector 64 [_RimColor] 4
Float 80 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_BumpMap] 2D 1
SetTexture 2 [unity_Lightmap] 2D 2
// 22 instructions, 3 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_1
eefieceddkpibkplnhpndgfodnpangnmkmdajigbabaaaaaaamagaaaaaeaaaaaa
daaaaaaacmacaaaafaafaaaaniafaaaaebgpgodjpeabaaaapeabaaaaaaacpppp
liabaaaadmaaaaaaabaadaaaaaaadmaaaaaadmaaadaaceaaaaaadmaaaaaaaaaa
abababaaacacacaaaaaaaeaaacaaaaaaaaaaaaaaaaacppppfbaaaaafacaaapka
aaaaaaeaaaaaialpaaaaaaaaaaaaiadpfbaaaaafadaaapkaaaaaaaebaaaaaaaa
aaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaaahla
bpaaaaacaaaaaaiaacaaadlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaaja
abaiapkabpaaaaacaaaaaajaacaiapkaabaaaaacaaaaabiaaaaakklaabaaaaac
aaaaaciaaaaapplaecaaaaadaaaacpiaaaaaoeiaabaioekaecaaaaadabaacpia
acaaoelaacaioekaecaaaaadacaacpiaaaaaoelaaaaioekaaeaaaaaeadaacbia
aaaappiaacaaaakaacaaffkaaeaaaaaeadaacciaaaaaffiaacaaaakaacaaffka
fkaaaaaeadaadiiaadaaoeiaadaaoeiaacaakkkaacaaaaadadaaciiaadaappib
acaappkaahaaaaacadaaciiaadaappiaagaaaaacadaaceiaadaappiaceaaaaac
aaaaahiaabaaoelaaiaaaaadaaaabbiaaaaaoeiaadaaoeiaacaaaaadaaaacbia
aaaaaaibacaappkacaaaaaadadaaabiaaaaaaaiaabaaaakaafaaaaadaaaachia
adaaaaiaaaaaoekaafaaaaadaaaaciiaabaappiaadaaaakaafaaaaadabaachia
abaaoeiaaaaappiaaeaaaaaeacaachiaacaaoeiaabaaoeiaaaaaoeiaabaaaaac
aaaicpiaacaaoeiappppaaaafdeieefcbmadaaaaeaaaaaaamhaaaaaafjaaaaae
egiocaaaaaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaa
abaaaaaafkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
gcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaaddcbabaaa
adaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegbcbaaa
acaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaa
aagabaaaabaaaaaadcaaaaapdcaabaaaabaaaaaahgapbaaaabaaaaaaaceaaaaa
aaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaa
aaaaaaaaapaaaaahicaabaaaaaaaaaaaegaabaaaabaaaaaaegaabaaaabaaaaaa
ddaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaai
icaabaaaaaaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaaf
ecaabaaaabaaaaaadkaabaaaaaaaaaaabacaaaahbcaabaaaaaaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaa
aaaaaaaaabeaaaaaaaaaiadpcpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaaibcaabaaaaaaaaaaaakaabaaaaaaaaaaaakiacaaaaaaaaaaaafaaaaaa
bjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaihcaabaaaaaaaaaaa
agaabaaaaaaaaaaaegiccaaaaaaaaaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaa
egbabaaaadaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaadiaaaaahicaabaaa
aaaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaebdiaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaapgapbaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadcaaaaajhccabaaaaaaaaaaa
egacbaaaacaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaa
aaaaaaaadkaabaaaacaaaaaadoaaaaabejfdeheoiaaaaaaaaeaaaaaaaiaaaaaa
giaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaadadaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaa
abaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaa
fdfgfpfegbhcghgfheaaklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES3"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_ShadowMapTexture] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 52 ALU, 3 TEX
PARAM c[6] = { program.local[0..4],
		{ 2, 1, 0, 48 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TXP R4.x, fragment.texcoord[4], texture[2], 2D;
TEX R1.yw, fragment.texcoord[0].zwzw, texture[1], 2D;
DP3 R1.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R1.z, R1.x;
MUL R2.xyz, R1.z, fragment.texcoord[1];
MAD R1.xy, R1.wyzw, c[5].x, -c[5].y;
MUL R1.zw, R1.xyxy, R1.xyxy;
ADD_SAT R1.w, R1.z, R1;
ADD R3.xyz, fragment.texcoord[2], R2;
DP3 R1.z, R3, R3;
RSQ R1.z, R1.z;
ADD R1.w, -R1, c[5].y;
MUL R3.xyz, R1.z, R3;
RSQ R1.w, R1.w;
RCP R1.z, R1.w;
DP3 R1.w, R1, R3;
MAX R1.w, R1, c[5].z;
POW R1.w, R1.w, c[5].w;
MUL R3.xyz, R1.w, c[0];
DP3 R1.w, R1, fragment.texcoord[2];
MUL R3.xyz, R3, R0.w;
MAX R1.w, R1, c[5].z;
MUL R4.yzw, R0.xxyz, c[0].xxyz;
MUL R4.yzw, R4, R1.w;
MUL R3.xyz, R3, c[2].x;
DP3 R1.w, R2, R2;
MUL R2.w, R4.x, c[5].x;
MAD R3.xyz, R3, c[1], R4.yzww;
MUL R3.xyz, R3, R2.w;
RSQ R2.w, R1.w;
MUL R2.xyz, R2.w, R2;
DP3 R1.w, fragment.texcoord[2], fragment.texcoord[2];
DP3 R2.w, R1, R2;
RSQ R1.w, R1.w;
MUL R2.xyz, R1.w, fragment.texcoord[2];
DP3 R2.x, R1, R2;
DP3 R1.w, fragment.texcoord[1], fragment.texcoord[1];
RSQ R1.w, R1.w;
ADD R2.w, -R2, R2.x;
MUL R2.xyz, R1.w, fragment.texcoord[1];
DP3_SAT R1.x, R1, R2;
ADD R1.w, R2, c[5].y;
POW R1.y, R1.w, c[4].x;
ADD R1.w, -R1.x, c[5].y;
MUL R1.xyz, R1.y, c[3];
MAD R2.xyz, R3, R1, R3;
POW R1.w, R1.w, c[4].x;
MUL R1.xyz, R1.w, c[3];
MAD R0.xyz, fragment.texcoord[3], R0, R2;
ADD result.color.xyz, R0, R1;
MOV result.color.w, R0;
END
# 52 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_ShadowMapTexture] 2D
"ps_2_0
; 62 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 48.00000000, 0, 0, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
dcl t4
texldp r6, t4, s2
texld r2, t0, s0
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s1
mov r1.x, r0.w
mov r1.y, r0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t1
mad_pp r4.xy, r1, c5.x, c5.y
add_pp r5.xyz, t2, r3
mul_pp r0.xy, r4, r4
add_pp_sat r0.x, r0, r0.y
dp3_pp r1.x, r5, r5
add_pp r0.x, -r0, c5.z
rsq_pp r0.x, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r5
rcp_pp r4.z, r0.x
dp3_pp r0.x, r4, r1
max_pp r0.x, r0, c5.w
pow r1.w, r0.x, c6.x
mov r0.x, r1.w
mul r0.xyz, r0.x, c0
mul r1.xyz, r0, r2.w
dp3_pp r0.x, r4, t2
max_pp r0.x, r0, c5.w
mul_pp r5.xyz, r2, c0
mul_pp r5.xyz, r5, r0.x
mul r1.xyz, r1, c2.x
mad r1.xyz, r1, c1, r5
mul_pp r0.x, r6, c5
mul r5.xyz, r1, r0.x
dp3_pp r1.x, r3, r3
dp3_pp r0.x, t2, t2
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r3
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t2
dp3_pp r0.x, r4, r1
dp3_pp r3.x, r4, r3
add_pp r0.x, -r0, r3
dp3 r1.x, t1, t1
rsq r1.x, r1.x
mul r3.xyz, r1.x, t1
add_pp r0.x, r0, c5.z
pow_pp r1.w, r0.x, c4.x
dp3_sat r0.x, r4, r3
add r0.x, -r0, c5.z
pow_pp r3.x, r0.x, c4.x
mul r0.xyz, r1.w, c3
mad r1.xyz, r5, r0, r5
mov_pp r0.x, r3.x
mad_pp r1.xyz, t3, r2, r1
mul r0.xyz, r0.x, c3
mov_pp r0.w, r2
add_pp r0.xyz, r1, r0
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 3 [_RimColor]
Float 4 [_RimPower]
Float 2 [_Shininess]
Vector 1 [_SpecColor]
SetTexture 0 [_ShadowMapTexture] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 45.33 (34 instructions), vertex: 0, texture: 12,
//   sequencer: 16, interpolator: 20;    10 GPRs, 18 threads,
// Performance (if enough threads): ~45 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaaboiaaaaacdiaaaaaaaaaaaaaaceaaaaabimaaaaableaaaaaaaa
aaaaaaaaaaaaabgeaaaaaabmaaaaabfhppppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabfaaaaaaalmaaadaaacaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaaa
aaabaaaaaaaaaaoiaaaaaaaaaaaaaapiaaadaaabaaabaaaaaaaaaamiaaaaaaaa
aaaaababaaacaaadaaabaaaaaaaaaaoiaaaaaaaaaaaaabalaaacaaaeaaabaaaa
aaaaabbiaaaaaaaaaaaaabciaaadaaaaaaabaaaaaaaaaamiaaaaaaaaaaaaabdk
aaacaaacaaabaaaaaaaaabbiaaaaaaaaaaaaabefaaacaaabaaabaaaaaaaaaaoi
aaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpemgjghgiheedgpgmgphcdaaaklklklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpengbgjgofegfhiaafpfcgjgnedgpgmgphcaafpfcgjgnfagphhgfhcaaklklkl
aaaaaaadaaabaaabaaabaaaaaaaaaaaafpfdgigbgegphhengbhafegfhihehfhc
gfaafpfdgigjgogjgogfhdhdaafpfdhagfgdedgpgmgphcaahahdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
abpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabpibaaaajaa
aaaaaaaeaaaaaaaaaaaaeekfaabpaabpaaaaaaabaaaapafaaaaahbfbaaaahcfc
aaaahdfdaaaapefeaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaadpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaaaaaaaaaa
lpiaaaaaeceaaaaaabfafaaeaaaabcaameaaaaaaaaaagaajgaapbcaabcaaaaaa
aaaagabfgablbcaabcaaaaaaaaaagacbcachbcaaccaaaaaaemiiacabaalolobl
paacacaemiadaaaeaabllaaaobacaeaabaaifaibbpbpppmhaaaaeaaadicieaab
bpbpppnjaaaaeaaababigaabbpbppgiiaaaaeaaamiabaaaaaaloloaapaababaa
beiaiaaaaaaaaablocaaaaagmiadaaahaagngmmgilaeppppfichaaaeaamamabl
kbagaaibfibnaaafaalbpagmobaaaciamiahaaajaagmmaaaobaaabaamjacaaaa
aalalalbnbahahppmiahaaabaamamaaaoaajacaaliceaaaaaaloloebnaajajpo
fiebaaaaaalolomgpaababiakaehahaiaamamglbobajaaiamiaeaaafaamploaa
paafahaamiacaaaaaaloloaapaahacaamjaeaaaaaaloloaapaajahaafibiaaaa
aalologmpaaiahiamiahaaabaamagmaaobabaaaamiabaaaaaaloloaapaabahaa
miajaaafaemhgmaakaaapoaaeaemaaabaakmlbblkcaappifeabdaaabaalalcmg
oaafafibeabcaaaaaagmblgmkbaappibdicjaaaaaagogmlbkbaaaeaadiihaaac
aalbmablkbaaaaaakibhafacaamabledmbacagaddibhaaacaamagmgmkbacacaa
kichafaaaagmmaidibaaadadmiahaaaaaaleleleolagadaakiehafacaamamamd
ibacabadmiahaaadaalelbaaobafabaamiahaaacaaleblleolaeabacmiahaaaa
aaleleleoladacaamiahiaaaaalelbmaolacabaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_ShadowMapTexture] 2D
"sce_fp_rsx // 61 instructions using 4 registers
[Configuration]
24
ffffffff0007c020001fffe0000000000000840004000000
[Offsets]
5
_LightColor0 2 0
0000021000000140
_SpecColor 1 0
00000230
_Shininess 1 0
000001e0
_RimColor 2 0
000003b000000310
_RimPower 2 0
00000370000002c0
[Microcode]
976
940017025c011c9dc8000001c8003fe106860440ce001c9d00020000aa020000
000040000000bf800000000000000000ce840140c8011c9dc8000001c8003fe1
1080b840c90c1c9dc90c0001c80000019e061700c8011c9dc8000001c8003fe1
ce803940c8011c9dc8000029c8003fe110800340c9001c9f00020000c8000001
00003f8000000000000000000000000008863b40ff003c9dff000001c8000001
02800540c90c1c9dc9000001c8000001ae823940c8011c9dc8000029c800bfe1
0e040340c9081c9dc9040001c80000010e883940c8081c9dc8000029c8000001
10800540c90c1c9dc9100001c800000110040900c9001c9d00020000c8000001
0000000000000000000000000000000004800540c90c1c9dc9080001c8000001
04001d00fe081c9dc8000001c80000010e880240c80c1c9dc8020001c8000001
0000000000000000000000000000000004020200c8001c9d00020000c8000001
0000424000000000000000000000000010800900ab001c9caa020000c8000001
000000000000000000000000000000000e823940c9041c9dc8000029c8000001
04800540c90c1c9dc9040001c800000104021c00aa041c9cc8000001c8000001
10000200c80c1c9daa040000c800000110000200c8001c9d00020000c8000001
000000000000000000000000000000000e880240c9101c9dff000001c8000001
1c000200fe001c9d20020001c800000100000000000000000000000000000000
0e040400f2001c9dc8021001c910000100000000000000000000000000000000
10800340ab001c9e01000000c800000102021805c8011c9dc8000001c8003fe1
10860340c9001c9d00020000c800000100003f80000000000000000000000000
0e000200c8081c9d00040000c8000001a4020500c8011c9dc8010001c800bfe1
02021d00ff0c1c9dc8000001c80000011000020000041c9c00020000c8000001
00000000000000000000000000000000ae043b00c8011c9daa040000c800bfe1
02048500c8081c9dc90c0001c800000104041c00fe001c9dc8000001c8000001
0e020200aa081c9cc8020001c800000100000000000000000000000000000000
1000030000081c9ec8020001c800000100000000000000000000000000003f80
0e800400c8001c9dc8040001c800000108001d00fe001c9dc8000001c8000001
1000020054001c9d00020000c800000100000000000000000000000000000000
ee800440c80c1c9dc8010001c9003fe110801c00fe001c9dc8000001c8000001
0e800440ff001c9dc8020001c900000100000000000000000000000000000000
10810140c80c1c9dc8000001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_BumpMap] 2D 2
SetTexture 2 [_ShadowMapTexture] 2D 0
// 52 instructions, 5 temp regs, 0 temp arrays:
// ALU 40 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecednbfhkompbdlmddefkmmbenmlnjphpbijabaaaaaakaahaaaaadaaaaaa
cmaaaaaaoeaaaaaabiabaaaaejfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaakeaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaakeaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapalaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklklfdeieefciaagaaaa
eaaaaaaakaabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaafkaaaaadaagabaaa
aaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagcbaaaad
lcbabaaaafaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaaaoaaaaah
dcaabaaaaaaaaaaaegbabaaaafaaaaaapgbpbaaaafaaaaaaefaaaaajpcaabaaa
aaaaaaaaegaabaaaaaaaaaaaeghobaaaacaaaaaaaagabaaaaaaaaaaaaaaaaaah
bcaabaaaaaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaabaaaaaahccaabaaa
aaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafccaabaaaaaaaaaaa
bkaabaaaaaaaaaaadcaaaaajhcaabaaaabaaaaaaegbcbaaaacaaaaaafgafbaaa
aaaaaaaaegbcbaaaadaaaaaadiaaaaahocaabaaaaaaaaaaafgafbaaaaaaaaaaa
agbjbaaaacaaaaaabaaaaaahicaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaa
abaaaaaaeeaaaaaficaabaaaabaaaaaadkaabaaaabaaaaaadiaaaaahhcaabaaa
abaaaaaapgapbaaaabaaaaaaegacbaaaabaaaaaaefaaaaajpcaabaaaacaaaaaa
ogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaadcaaaaapdcaabaaa
acaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaaapaaaaahicaabaaaabaaaaaa
egaabaaaacaaaaaaegaabaaaacaaaaaaddaaaaahicaabaaaabaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaaabaaaaaadkaabaiaebaaaaaa
abaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaaacaaaaaadkaabaaaabaaaaaa
baaaaaahbcaabaaaabaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaadeaaaaah
bcaabaaaabaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaa
abaaaaaaakaabaaaabaaaaaadiaaaaahbcaabaaaabaaaaaaakaabaaaabaaaaaa
abeaaaaaaaaaeaecbjaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaagaabaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaaefaaaaaj
pcaabaaaadaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaa
diaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaadaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaagiacaaaaaaaaaaaahaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaacaaaaaabaaaaaah
icaabaaaabaaaaaaegacbaaaacaaaaaaegbcbaaaadaaaaaadeaaaaahicaabaaa
abaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaaadiaaaaaihcaabaaaaeaaaaaa
egacbaaaadaaaaaaegiccaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaaabaaaaaa
egacbaaaaeaaaaaapgapbaaaabaaaaaaegacbaaaabaaaaaadiaaaaahhcaabaaa
abaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbcaabaaaaaaaaaaa
jgahbaaaaaaaaaaaegacbaaaacaaaaaaaaaaaaaiccaabaaaaaaaaaaaakaabaia
ebaaaaaaaaaaaaaaabeaaaaaaaaaiadpdgcaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaaabeaaaaa
aaaaiadpcpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaibcaabaaa
aaaaaaaaakaabaaaaaaaaaaaakiacaaaaaaaaaaaajaaaaaabjaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaabaaaaaahecaabaaaaaaaaaaaegbcbaaaadaaaaaa
egbcbaaaadaaaaaaeeaaaaafecaabaaaaaaaaaaackaabaaaaaaaaaaadiaaaaah
hcaabaaaaeaaaaaakgakbaaaaaaaaaaaegbcbaaaadaaaaaabaaaaaahecaabaaa
aaaaaaaaegacbaaaaeaaaaaaegacbaaaacaaaaaaaaaaaaahccaabaaaaaaaaaaa
ckaabaaaaaaaaaaabkaabaaaaaaaaaaacpaaaaafccaabaaaaaaaaaaabkaabaaa
aaaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaaaaaaaaa
ajaaaaaabjaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaaiocaabaaa
aaaaaaaafgafbaaaaaaaaaaaagijcaaaaaaaaaaaaiaaaaaadcaaaaajocaabaaa
aaaaaaaaagajbaaaabaaaaaafgaobaaaaaaaaaaaagajbaaaabaaaaaadcaaaaaj
ocaabaaaaaaaaaaaagajbaaaadaaaaaaagbjbaaaaeaaaaaafgaobaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaadkaabaaaadaaaaaadcaaaaakhccabaaaaaaaaaaa
egiccaaaaaaaaaaaaiaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_ShadowMapTexture] 2D
"agal_ps
c5 2.0 -1.0 1.0 0.0
c6 48.0 0.0 0.0 0.0
[bc]
aeaaaaaaaaaaapacaeaaaaoeaeaaaaaaaeaaaappaeaaaaaa div r0, v4, v4.w
ciaaaaaaagaaapacaaaaaafeacaaaaaaacaaaaaaafaababb tex r6, r0.xyyy, s2 <2d wrap linear point>
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
ciaaaaaaaaaaapacaaaaaafeacaaaaaaabaaaaaaafaababb tex r0, r0.xyyy, s1 <2d wrap linear point>
aaaaaaaaabaaabacaaaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r0.w
aaaaaaaaabaaacacaaaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r1.y, r0.y
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r0.x, v1
adaaaaaaaeaaadacabaaaafeacaaaaaaafaaaaaaabaaaaaa mul r4.xy, r1.xyyy, c5.x
abaaaaaaaeaaadacaeaaaafeacaaaaaaafaaaaffabaaaaaa add r4.xy, r4.xyyy, c5.y
abaaaaaaafaaahacacaaaaoeaeaaaaaaadaaaakeacaaaaaa add r5.xyz, v2, r3.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r5.xyzz
adaaaaaaaaaaabacaeaaaaffacaaaaaaaeaaaaffacaaaaaa mul r0.x, r4.y, r4.y
bfaaaaaaabaaaiacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.w, r4.x
adaaaaaaabaaaiacabaaaappacaaaaaaaeaaaaaaacaaaaaa mul r1.w, r1.w, r4.x
acaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.w, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaafaaaakeacaaaaaa mul r1.xyz, r1.x, r5.xyzz
afaaaaaaaeaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r4.z, r0.x
bcaaaaaaaaaaabacaeaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r4.xyzz, r1.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
alaaaaaaabaaapacaaaaaaaaacaaaaaaagaaaaaaabaaaaaa pow r1, r0.x, c6.x
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaaaaaaaoeabaaaaaa mul r0.xyz, r0.x, c0
adaaaaaaabaaahacaaaaaakeacaaaaaaacaaaappacaaaaaa mul r1.xyz, r0.xyzz, r2.w
bcaaaaaaaaaaabacaeaaaakeacaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, r4.xyzz, v2
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
adaaaaaaafaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r5.xyz, r2.xyzz, c0
adaaaaaaafaaahacafaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r5.xyz, r5.xyzz, r0.x
adaaaaaaabaaahacabaaaakeacaaaaaaacaaaaaaabaaaaaa mul r1.xyz, r1.xyzz, c2.x
adaaaaaaabaaahacabaaaakeacaaaaaaabaaaaoeabaaaaaa mul r1.xyz, r1.xyzz, c1
abaaaaaaabaaahacabaaaakeacaaaaaaafaaaakeacaaaaaa add r1.xyz, r1.xyzz, r5.xyzz
adaaaaaaaaaaabacagaaaaaaacaaaaaaafaaaaoeabaaaaaa mul r0.x, r6.x, c5
adaaaaaaafaaahacabaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r5.xyz, r1.xyzz, r0.x
bcaaaaaaabaaabacadaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r1.x, r3.xyzz, r3.xyzz
bcaaaaaaaaaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, v2, v2
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaadaaaakeacaaaaaa mul r1.xyz, r1.x, r3.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r3.xyz, r0.x, v2
bcaaaaaaaaaaabacaeaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r4.xyzz, r1.xyzz
bcaaaaaaadaaabacaeaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r3.x, r4.xyzz, r3.xyzz
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaadaaaaaaacaaaaaa add r0.x, r0.x, r3.x
bcaaaaaaabaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, v1, v1
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaadaaahacabaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r1.x, v1
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
alaaaaaaabaaapacaaaaaaaaacaaaaaaaeaaaaaaabaaaaaa pow r1, r0.x, c4.x
bcaaaaaaaaaaabacaeaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r0.x, r4.xyzz, r3.xyzz
bgaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa sat r0.x, r0.x
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
alaaaaaaadaaapacaaaaaaaaacaaaaaaaeaaaaaaabaaaaaa pow r3, r0.x, c4.x
adaaaaaaaaaaahacabaaaaaaacaaaaaaadaaaaoeabaaaaaa mul r0.xyz, r1.x, c3
adaaaaaaabaaahacafaaaakeacaaaaaaaaaaaakeacaaaaaa mul r1.xyz, r5.xyzz, r0.xyzz
abaaaaaaabaaahacabaaaakeacaaaaaaafaaaakeacaaaaaa add r1.xyz, r1.xyzz, r5.xyzz
aaaaaaaaaaaaabacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r3.x
adaaaaaaadaaahacadaaaaoeaeaaaaaaacaaaakeacaaaaaa mul r3.xyz, v3, r2.xyzz
abaaaaaaabaaahacadaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r3.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaaaaacaaaaaaadaaaaoeabaaaaaa mul r0.xyz, r0.x, c3
aaaaaaaaaaaaaiacacaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.w, r2.w
abaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa add r0.xyz, r1.xyzz, r0.xyzz
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_BumpMap] 2D 2
SetTexture 2 [_ShadowMapTexture] 2D 0
// 52 instructions, 5 temp regs, 0 temp arrays:
// ALU 40 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_1
eefiecedbefmgaegdcadjlhoejepkjplnfmdpanoabaaaaaagaalaaaaaeaaaaaa
daaaaaaaomadaaaaheakaaaacmalaaaaebgpgodjleadaaaaleadaaaaaaacpppp
gmadaaaaeiaaaaaaacaadaaaaaaaeiaaaaaaeiaaadaaceaaaaaaeiaaacaaaaaa
aaababaaabacacaaaaaaabaaacaaaaaaaaaaaaaaaaaaahaaadaaacaaaaaaaaaa
aaacppppfbaaaaafafaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaiadpfbaaaaaf
agaaapkaaaaaeaecaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaapla
bpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaacaaaaaaia
adaachlabpaaaaacaaaaaaiaaeaaaplabpaaaaacaaaaaajaaaaiapkabpaaaaac
aaaaaajaabaiapkabpaaaaacaaaaaajaacaiapkaagaaaaacaaaaaiiaaeaappla
afaaaaadaaaaadiaaaaappiaaeaaoelaabaaaaacabaaabiaaaaakklaabaaaaac
abaaaciaaaaapplaecaaaaadaaaacpiaaaaaoeiaaaaioekaecaaaaadabaacpia
abaaoeiaacaioekaecaaaaadacaaapiaaaaaoelaabaioekaacaaaaadaaaaabia
aaaaaaiaaaaaaaiaaiaaaaadaaaaaciaabaaoelaabaaoelaahaaaaacaaaaacia
aaaaffiaabaaaaacadaaahiaabaaoelaaeaaaaaeadaachiaadaaoeiaaaaaffia
acaaoelaafaaaaadaeaaahiaaaaaffiaabaaoelaceaaaaacafaachiaadaaoeia
aeaaaaaeadaacbiaabaappiaafaaaakaafaaffkaaeaaaaaeadaacciaabaaffia
afaaaakaafaaffkafkaaaaaeadaadiiaadaaoeiaadaaoeiaafaakkkaacaaaaad
adaaciiaadaappibafaappkaahaaaaacadaaciiaadaappiaagaaaaacadaaceia
adaappiaaiaaaaadadaaciiaadaaoeiaafaaoeiaalaaaaadaeaaaiiaadaappia
afaakkkacaaaaaadadaaaiiaaeaappiaagaaaakaafaaaaadaaaaaoiaadaappia
aaaablkaafaaaaadaaaaaoiaacaappiaaaaaoeiaafaaaaadaaaaaoiaaaaaoeia
acaaaakaafaaaaadaaaaaoiaaaaaoeiaabaablkaaiaaaaadadaaciiaadaaoeia
acaaoelaalaaaaadaeaaciiaadaappiaafaakkkaafaaaaadabaachiaacaaoeia
aaaaoekaaeaaaaaeaaaaaoiaabaabliaaeaappiaaaaaoeiaafaaaaadaaaachia
aaaaaaiaaaaabliaaiaaaaadaaaaaiiaaeaaoeiaadaaoeiaacaaaaadadaaciia
aaaappibafaappkaabaaaaacaaaabiiaaaaappiaacaaaaadaaaaciiaaaaappib
afaappkacaaaaaadabaaabiaaaaappiaaeaaaakaceaaaaacaeaachiaacaaoela
aiaaaaadaaaaciiaaeaaoeiaadaaoeiaacaaaaadaaaaciiaaaaappiaadaappia
caaaaaadabaaaciaaaaappiaaeaaaakaafaaaaadabaaaoiaabaaffiaadaablka
aeaaaaaeaaaachiaaaaaoeiaabaabliaaaaaoeiaaeaaaaaeaaaachiaacaaoeia
adaaoelaaaaaoeiaaeaaaaaeacaachiaadaaoekaabaaaaiaaaaaoeiaabaaaaac
aaaicpiaacaaoeiappppaaaafdeieefciaagaaaaeaaaaaaakaabaaaafjaaaaae
egiocaaaaaaaaaaaakaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaa
abaaaaaafkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
gcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaa
adaaaaaagcbaaaadhcbabaaaaeaaaaaagcbaaaadlcbabaaaafaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacafaaaaaaaoaaaaahdcaabaaaaaaaaaaaegbabaaa
afaaaaaapgbpbaaaafaaaaaaefaaaaajpcaabaaaaaaaaaaaegaabaaaaaaaaaaa
eghobaaaacaaaaaaaagabaaaaaaaaaaaaaaaaaahbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaakaabaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaa
egbcbaaaacaaaaaaeeaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadcaaaaaj
hcaabaaaabaaaaaaegbcbaaaacaaaaaafgafbaaaaaaaaaaaegbcbaaaadaaaaaa
diaaaaahocaabaaaaaaaaaaafgafbaaaaaaaaaaaagbjbaaaacaaaaaabaaaaaah
icaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaabaaaaaaeeaaaaaficaabaaa
abaaaaaadkaabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaapgapbaaaabaaaaaa
egacbaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaacaaaaaadcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaa
aceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialp
aaaaaaaaaaaaaaaaapaaaaahicaabaaaabaaaaaaegaabaaaacaaaaaaegaabaaa
acaaaaaaddaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaiadp
aaaaaaaiicaabaaaabaaaaaadkaabaiaebaaaaaaabaaaaaaabeaaaaaaaaaiadp
elaaaaafecaabaaaacaaaaaadkaabaaaabaaaaaabaaaaaahbcaabaaaabaaaaaa
egacbaaaacaaaaaaegacbaaaabaaaaaadeaaaaahbcaabaaaabaaaaaaakaabaaa
abaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaa
diaaaaahbcaabaaaabaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaeaecbjaaaaaf
bcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaagaabaaa
abaaaaaaegiccaaaaaaaaaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaapgapbaaaadaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaagiacaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaacaaaaaabaaaaaahicaabaaaabaaaaaaegacbaaa
acaaaaaaegbcbaaaadaaaaaadeaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaa
abeaaaaaaaaaaaaadiaaaaaihcaabaaaaeaaaaaaegacbaaaadaaaaaaegiccaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaabaaaaaaegacbaaaaeaaaaaapgapbaaa
abaaaaaaegacbaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbcaabaaaaaaaaaaajgahbaaaaaaaaaaaegacbaaa
acaaaaaaaaaaaaaiccaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaaabeaaaaa
aaaaiadpdgcaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaaaaaaaaibcaabaaa
aaaaaaaaakaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpcpaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaaibcaabaaaaaaaaaaaakaabaaaaaaaaaaa
akiacaaaaaaaaaaaajaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
baaaaaahecaabaaaaaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaaf
ecaabaaaaaaaaaaackaabaaaaaaaaaaadiaaaaahhcaabaaaaeaaaaaakgakbaaa
aaaaaaaaegbcbaaaadaaaaaabaaaaaahecaabaaaaaaaaaaaegacbaaaaeaaaaaa
egacbaaaacaaaaaaaaaaaaahccaabaaaaaaaaaaackaabaaaaaaaaaaabkaabaaa
aaaaaaaacpaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaaiccaabaaa
aaaaaaaabkaabaaaaaaaaaaaakiacaaaaaaaaaaaajaaaaaabjaaaaafccaabaaa
aaaaaaaabkaabaaaaaaaaaaadiaaaaaiocaabaaaaaaaaaaafgafbaaaaaaaaaaa
agijcaaaaaaaaaaaaiaaaaaadcaaaaajocaabaaaaaaaaaaaagajbaaaabaaaaaa
fgaobaaaaaaaaaaaagajbaaaabaaaaaadcaaaaajocaabaaaaaaaaaaaagajbaaa
adaaaaaaagbjbaaaaeaaaaaafgaobaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaa
dkaabaaaadaaaaaadcaaaaakhccabaaaaaaaaaaaegiccaaaaaaaaaaaaiaaaaaa
agaabaaaaaaaaaaajgahbaaaaaaaaaaadoaaaaabejfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahahaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahahaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapalaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES3"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_ShadowMapTexture] 2D
SetTexture 3 [unity_Lightmap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 26 ALU, 4 TEX
PARAM c[3] = { program.local[0..1],
		{ 8, 2, 1 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R2.yw, fragment.texcoord[0].zwzw, texture[1], 2D;
TEX R1, fragment.texcoord[2], texture[3], 2D;
TXP R3.x, fragment.texcoord[3], texture[2], 2D;
MAD R2.xy, R2.wyzw, c[2].y, -c[2].z;
MUL R2.zw, R2.xyxy, R2.xyxy;
ADD_SAT R2.z, R2, R2.w;
ADD R2.z, -R2, c[2];
DP3 R2.w, fragment.texcoord[1], fragment.texcoord[1];
RSQ R2.w, R2.w;
RSQ R2.z, R2.z;
MUL R3.yzw, R2.w, fragment.texcoord[1].xxyz;
RCP R2.z, R2.z;
DP3_SAT R2.x, R3.yzww, R2;
ADD R2.w, -R2.x, c[2].z;
MUL R3.yzw, R1.w, R1.xxyz;
MUL R2.xyz, R1, R3.x;
MUL R1.xyz, R2, c[2].y;
MUL R2.xyz, R3.yzww, c[2].x;
MIN R1.xyz, R2, R1;
MUL R3.xyz, R2, R3.x;
POW R1.w, R2.w, c[1].x;
MUL R2.xyz, R1.w, c[0];
MAX R1.xyz, R1, R3;
MAD result.color.xyz, R0, R1, R2;
MOV result.color.w, R0;
END
# 26 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_ShadowMapTexture] 2D
SetTexture 3 [unity_Lightmap] 2D
"ps_2_0
; 29 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
def c2, 8.00000000, 2.00000000, -1.00000000, 1.00000000
dcl t0
dcl t1.xyz
dcl t2.xy
dcl t3
texld r3, t0, s0
texld r2, t2, s3
texldp r4, t3, s2
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s1
mov r0.x, r0.w
mad_pp r5.xy, r0, c2.y, c2.z
mul_pp r0.xy, r5, r5
add_pp_sat r0.x, r0, r0.y
add_pp r1.x, -r0, c2.w
rsq_pp r1.x, r1.x
rcp_pp r5.z, r1.x
mul_pp r1.xyz, r2, r4.x
dp3 r0.x, t1, t1
rsq r0.x, r0.x
mul r0.xyz, r0.x, t1
dp3_sat r0.x, r0, r5
mul_pp r2.xyz, r2.w, r2
mul_pp r2.xyz, r2, c2.x
mul_pp r1.xyz, r1, c2.y
min_pp r5.xyz, r2, r1
add r0.x, -r0, c2.w
pow_pp r1.x, r0.x, c1.x
mul_pp r0.xyz, r2, r4.x
max_pp r2.xyz, r5, r0
mov_pp r0.x, r1.x
mul r0.xyz, r0.x, c0
mov_pp r0.w, r3
mad_pp r0.xyz, r3, r2, r0
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_ShadowMapTexture] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [unity_Lightmap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 24.00 (18 instructions), vertex: 0, texture: 16,
//   sequencer: 10, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~24 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabkeaaaaabhiaaaaaaaaaaaaaaceaaaaabemaaaaabheaaaaaaaa
aaaaaaaaaaaaabceaaaaaabmaaaaabbippppadaaaaaaaaagaaaaaabmaaaaaaaa
aaaaabbbaaaaaajeaaadaaacaaabaaaaaaaaaakaaaaaaaaaaaaaaalaaaadaaab
aaabaaaaaaaaaakaaaaaaaaaaaaaaaljaaacaaaaaaabaaaaaaaaaameaaaaaaaa
aaaaaaneaaacaaabaaabaaaaaaaaaaoaaaaaaaaaaaaaaapaaaadaaaaaaabaaaa
aaaaaakaaaaaaaaaaaaaabacaaadaaadaaabaaaaaaaaaakaaaaaaaaafpechfgn
haengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpengbgjgofegfhi
aafpfcgjgnedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpfcgjgn
fagphhgfhcaaklklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpfdgigbgegphhen
gbhafegfhihehfhcgfaahfgogjhehjfpemgjghgihegngbhaaahahdfpddfpdaaa
dccodacodcdadddfddcodaaaaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
abpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabdibaaaagaa
aaaaaaaeaaaaaaaaaaaadeieaaapaaapaaaaaaabaaaapafaaaaahbfbaaaadcfc
aaaapdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaalpiaaaaaaaaaaaaadpiaaaaa
ebaaaaaaaffagaadaaaabcaameaaaaaaaaaagaajgaapbcaabcaaaaaaaaaaeabf
aaaaccaaaaaaaaaaemeiacabaaloloblpaababadmiamaaacaamgkmaaobacadaa
liaieaebbpbppppiaaaaeaaadicieaabbpbppompaaaaeaaabadigaebbpbppgii
aaaaeaaababicaabbpbppgiiaaaaeaaabeiaiaaaaaaaaablocaaaaacficbaaaa
aablblblkbagppibaachaaafaalbmagmobaaabaeaaehaaabaagmmamgobaaagae
aaihaaadaamagmlbobabaeaemiadaaaeaabkgmaakaaappaamjabaaaaaalalalb
nbaeaepplibaaaaaaaaaaamamcaaaappkaeoaeaaaalbpmgmobaaagiamjabaaaa
aaloloaapaafaeaalibaaaaaaaaaaamamcaaaappeachaaabaamabfgmodabaaia
kibaaaaaaaaaaaebmcaaaaabdibhaaabaamamagmocadabaamiahaaaaaagmmaaa
kbaaaaaamiahiaaaaamamamaolabacaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_ShadowMapTexture] 2D
SetTexture 3 [unity_Lightmap] 2D
"sce_fp_rsx // 28 instructions using 3 registers
[Configuration]
24
ffffffff0003c020000ffff4000000000000840003000000
[Offsets]
2
_RimColor 1 0
000001b0
_RimPower 1 0
00000150
[Microcode]
448
940017025c011c9dc8000001c8003fe106800440ce001c9d00020000aa020000
000040000000bf800000000000000000a8000500c8011c9dc8010001c800bfe1
0880b840c9001c9dc9000001c80000011080034055001c9f00020000c8000001
00003f80000000000000000000000000ae023b00c8011c9d54000001c800bfe1
08803b40ff003c9dff000001c800000104048500c8041c9dc9000001c8000001
de001706c8011c9dc8000001c8003fe10e840240fe001c9dc8003001c8000001
e2041804c8011c9dc8000001c8003fe10e860240c8001c9d00081000c8000001
10040300aa081c9e00020000c800000100003f80000000000000000000000000
9e001700c8011c9dc8000001c8003fe108041d00fe081c9dc8000001c8000001
0e880240c9081c9d00080000c80000010e840840c9081c9dc90c0001c8000001
1002020054081c9d00020000c800000100000000000000000000000000000000
0e880940c9081c9dc9100001c800000102841c00fe041c9dc8000001c8000001
0e800240c8001c9dc9100001c800000110800140c8001c9dc8000001c8000001
0e81044001081c9cc8020001c900000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 208 // 148 used size, 10 vars
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_BumpMap] 2D 2
SetTexture 2 [_ShadowMapTexture] 2D 0
SetTexture 3 [unity_Lightmap] 2D 3
// 29 instructions, 3 temp regs, 0 temp arrays:
// ALU 21 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedbibicomljjfokbjkmjnbdgijfbannapdabaaaaaabiafaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaadadaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapalaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcbaaeaaaaeaaaaaaaaeabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaaddcbabaaaadaaaaaagcbaaaadlcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegbcbaaaacaaaaaa
efaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
acaaaaaadcaaaaapdcaabaaaabaaaaaahgapbaaaabaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
apaaaaahicaabaaaaaaaaaaaegaabaaaabaaaaaaegaabaaaabaaaaaaddaaaaah
icaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaa
aaaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaa
abaaaaaadkaabaaaaaaaaaaabacaaaahbcaabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaa
abeaaaaaaaaaiadpcpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaai
bcaabaaaaaaaaaaaakaabaaaaaaaaaaaakiacaaaaaaaaaaaajaaaaaabjaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaihcaabaaaaaaaaaaaagaabaaa
aaaaaaaaegiccaaaaaaaaaaaaiaaaaaaaoaaaaahdcaabaaaabaaaaaaegbabaaa
aeaaaaaapgbpbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaaabaaaaaa
eghobaaaacaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaaaaaaaaaaakaabaaa
abaaaaaaakaabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaadaaaaaa
eghobaaaadaaaaaaaagabaaaadaaaaaadiaaaaahocaabaaaabaaaaaapgapbaaa
aaaaaaaaagajbaaaacaaaaaadiaaaaahicaabaaaaaaaaaaadkaabaaaacaaaaaa
abeaaaaaaaaaaaebdiaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaapgapbaaa
aaaaaaaaddaaaaahocaabaaaabaaaaaafgaobaaaabaaaaaaagajbaaaacaaaaaa
diaaaaahhcaabaaaacaaaaaaagaabaaaabaaaaaaegacbaaaacaaaaaadeaaaaah
hcaabaaaabaaaaaajgahbaaaabaaaaaaegacbaaaacaaaaaaefaaaaajpcaabaaa
acaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaadcaaaaaj
hccabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaadkaabaaaacaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_RimColor]
Float 1 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_ShadowMapTexture] 2D
SetTexture 3 [unity_Lightmap] 2D
"agal_ps
c2 8.0 2.0 -1.0 1.0
[bc]
ciaaaaaaadaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r3, v0, s0 <2d wrap linear point>
ciaaaaaaacaaapacacaaaaoeaeaaaaaaadaaaaaaafaababb tex r2, v2, s3 <2d wrap linear point>
aeaaaaaaaaaaapacadaaaaoeaeaaaaaaadaaaappaeaaaaaa div r0, v3, v3.w
ciaaaaaaaeaaapacaaaaaafeacaaaaaaacaaaaaaafaababb tex r4, r0.xyyy, s2 <2d wrap linear point>
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
ciaaaaaaaaaaapacaaaaaafeacaaaaaaabaaaaaaafaababb tex r0, r0.xyyy, s1 <2d wrap linear point>
aaaaaaaaaaaaabacaaaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r0.w
adaaaaaaafaaadacaaaaaafeacaaaaaaacaaaaffabaaaaaa mul r5.xy, r0.xyyy, c2.y
abaaaaaaafaaadacafaaaafeacaaaaaaacaaaakkabaaaaaa add r5.xy, r5.xyyy, c2.z
adaaaaaaaaaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r0.x, r5.y, r5.y
bfaaaaaaabaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.x, r5.x
adaaaaaaabaaabacabaaaaaaacaaaaaaafaaaaaaacaaaaaa mul r1.x, r1.x, r5.x
acaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.x, r0.x
abaaaaaaabaaabacaaaaaaaaacaaaaaaacaaaappabaaaaaa add r1.x, r0.x, c2.w
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
afaaaaaaafaaaeacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r1.x
adaaaaaaabaaahacacaaaakeacaaaaaaaeaaaaaaacaaaaaa mul r1.xyz, r2.xyzz, r4.x
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r0.xyz, r0.x, v1
bcaaaaaaaaaaabacaaaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r0.x, r0.xyzz, r5.xyzz
bgaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa sat r0.x, r0.x
adaaaaaaacaaahacacaaaappacaaaaaaacaaaakeacaaaaaa mul r2.xyz, r2.w, r2.xyzz
adaaaaaaacaaahacacaaaakeacaaaaaaacaaaaaaabaaaaaa mul r2.xyz, r2.xyzz, c2.x
adaaaaaaabaaahacabaaaakeacaaaaaaacaaaaffabaaaaaa mul r1.xyz, r1.xyzz, c2.y
agaaaaaaafaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa min r5.xyz, r2.xyzz, r1.xyzz
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaacaaaappabaaaaaa add r0.x, r0.x, c2.w
alaaaaaaabaaapacaaaaaaaaacaaaaaaabaaaaaaabaaaaaa pow r1, r0.x, c1.x
adaaaaaaaaaaahacacaaaakeacaaaaaaaeaaaaaaacaaaaaa mul r0.xyz, r2.xyzz, r4.x
ahaaaaaaacaaahacafaaaakeacaaaaaaaaaaaakeacaaaaaa max r2.xyz, r5.xyzz, r0.xyzz
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaaaaaaaoeabaaaaaa mul r0.xyz, r0.x, c0
aaaaaaaaaaaaaiacadaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.w, r3.w
adaaaaaaabaaahacadaaaakeacaaaaaaacaaaakeacaaaaaa mul r1.xyz, r3.xyzz, r2.xyzz
abaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa add r0.xyz, r1.xyzz, r0.xyzz
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 208 // 148 used size, 10 vars
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_BumpMap] 2D 2
SetTexture 2 [_ShadowMapTexture] 2D 0
SetTexture 3 [unity_Lightmap] 2D 3
// 29 instructions, 3 temp regs, 0 temp arrays:
// ALU 21 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_1
eefiecedofbmcjegojhdjddfdmkboancjehhehbnabaaaaaalaahaaaaaeaaaaaa
daaaaaaameacaaaanmagaaaahmahaaaaebgpgodjimacaaaaimacaaaaaaacpppp
emacaaaaeaaaaaaaabaadeaaaaaaeaaaaaaaeaaaaeaaceaaaaaaeaaaacaaaaaa
aaababaaabacacaaadadadaaaaaaaiaaacaaaaaaaaaaaaaaaaacppppfbaaaaaf
acaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaiadpfbaaaaafadaaapkaaaaaaaeb
aaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaia
abaaahlabpaaaaacaaaaaaiaacaaadlabpaaaaacaaaaaaiaadaaaplabpaaaaac
aaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaacaaaaaajaacaiapka
bpaaaaacaaaaaajaadaiapkaabaaaaacaaaaabiaaaaakklaabaaaaacaaaaacia
aaaapplaagaaaaacaaaaaeiaadaapplaafaaaaadabaaadiaaaaakkiaadaaoela
ecaaaaadaaaacpiaaaaaoeiaacaioekaecaaaaadabaacpiaabaaoeiaaaaioeka
ecaaaaadacaacpiaacaaoelaadaioekaecaaaaadadaacpiaaaaaoelaabaioeka
aeaaaaaeaeaacbiaaaaappiaacaaaakaacaaffkaaeaaaaaeaeaacciaaaaaffia
acaaaakaacaaffkafkaaaaaeaeaadiiaaeaaoeiaaeaaoeiaacaakkkaacaaaaad
aeaaciiaaeaappibacaappkaahaaaaacaeaaciiaaeaappiaagaaaaacaeaaceia
aeaappiaceaaaaacaaaaahiaabaaoelaaiaaaaadaaaabbiaaaaaoeiaaeaaoeia
acaaaaadaaaacbiaaaaaaaibacaappkacaaaaaadabaaaciaaaaaaaiaabaaaaka
afaaaaadaaaachiaabaaffiaaaaaoekaacaaaaadaaaaciiaabaaaaiaabaaaaia
afaaaaadabaacoiaacaabliaaaaappiaafaaaaadaaaaciiaacaappiaadaaaaka
afaaaaadacaachiaacaaoeiaaaaappiaakaaaaadaeaachiaabaabliaacaaoeia
afaaaaadabaachiaabaaaaiaacaaoeiaalaaaaadacaachiaaeaaoeiaabaaoeia
aeaaaaaeadaachiaadaaoeiaacaaoeiaaaaaoeiaabaaaaacaaaicpiaadaaoeia
ppppaaaafdeieefcbaaeaaaaeaaaaaaaaeabaaaafjaaaaaeegiocaaaaaaaaaaa
akaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaad
aagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaa
ffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaa
ffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaa
gcbaaaadhcbabaaaacaaaaaagcbaaaaddcbabaaaadaaaaaagcbaaaadlcbabaaa
aeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegbcbaaa
acaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaa
aagabaaaacaaaaaadcaaaaapdcaabaaaabaaaaaahgapbaaaabaaaaaaaceaaaaa
aaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaa
aaaaaaaaapaaaaahicaabaaaaaaaaaaaegaabaaaabaaaaaaegaabaaaabaaaaaa
ddaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaai
icaabaaaaaaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaaf
ecaabaaaabaaaaaadkaabaaaaaaaaaaabacaaaahbcaabaaaaaaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaa
aaaaaaaaabeaaaaaaaaaiadpcpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaaibcaabaaaaaaaaaaaakaabaaaaaaaaaaaakiacaaaaaaaaaaaajaaaaaa
bjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaihcaabaaaaaaaaaaa
agaabaaaaaaaaaaaegiccaaaaaaaaaaaaiaaaaaaaoaaaaahdcaabaaaabaaaaaa
egbabaaaaeaaaaaapgbpbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaa
abaaaaaaeghobaaaacaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaaaaaaaaaa
akaabaaaabaaaaaaakaabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaa
adaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaadiaaaaahocaabaaaabaaaaaa
pgapbaaaaaaaaaaaagajbaaaacaaaaaadiaaaaahicaabaaaaaaaaaaadkaabaaa
acaaaaaaabeaaaaaaaaaaaebdiaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaa
pgapbaaaaaaaaaaaddaaaaahocaabaaaabaaaaaafgaobaaaabaaaaaaagajbaaa
acaaaaaadiaaaaahhcaabaaaacaaaaaaagaabaaaabaaaaaaegacbaaaacaaaaaa
deaaaaahhcaabaaaabaaaaaajgahbaaaabaaaaaaegacbaaaacaaaaaaefaaaaaj
pcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaa
dcaaaaajhccabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaaegacbaaa
aaaaaaaadgaaaaaficcabaaaaaaaaaaadkaabaaaacaaaaaadoaaaaabejfdeheo
jiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaa
apaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaaimaaaaaa
abaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaa
adaaaaaaadaaaaaaadadaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaa
apalaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheo
cmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaa
apaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES3"
}

}
	}
	Pass {
		Name "FORWARD"
		Tags { "LightMode" = "ForwardAdd" }
		ZWrite Off Blend One One Fog { Color (0,0,0,0) }
Program "vp" {
// Vertex combos: 5
//   opengl - ALU: 26 to 35
//   d3d9 - ALU: 29 to 38
//   d3d11 - ALU: 12 to 14, TEX: 0 to 0, FLOW: 1 to 1
//   d3d11_9x - ALU: 12 to 14, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_WorldSpaceLightPos0]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 19 [unity_Scale]
Matrix 13 [_LightMatrix0]
Vector 20 [_MainTex_ST]
Vector 21 [_BumpMap_ST]
"!!ARBvp1.0
# 34 ALU
PARAM c[22] = { { 1 },
		state.matrix.mvp,
		program.local[5..21] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[18];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.x, R0, c[9];
DP4 R3.y, R0, c[10];
MAD R0.xyz, R3, c[19].w, -vertex.position;
DP3 result.texcoord[1].y, R0, R1;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.w, vertex.position, c[8];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
DP4 result.texcoord[3].z, R0, c[15];
DP4 result.texcoord[3].y, R0, c[14];
DP4 result.texcoord[3].x, R0, c[13];
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[21].xyxy, c[21];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 34 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
Vector 20 [_BumpMap_ST]
"vs_2_0
; 37 ALU
def c21, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c21.x
mov r0.xyz, c16
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c18.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c17, r0
mov r0, c9
dp4 r4.y, c17, r0
mov r1, c8
dp4 r4.x, c17, r1
mad r0.xyz, r4, c18.w, -v0
dp3 oT1.y, r0, r2
dp3 oT1.z, v2, r0
dp3 oT1.x, r0, v1
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 oT2.y, r2, r3
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
dp4 oT3.z, r0, c14
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.zw, v3.xyxy, c20.xyxy, c20
mad oT0.xy, v3, c19, c19.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 20 [_BumpMap_ST]
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 42.67 (32 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacdiaaaaabpiaaaaaaaaaaaaaaceaaaaaaaaaaaaabmmaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaabeaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaap
aaaeaaaaaaaaaapmaaaaaaaaaaaaabamaaacaabdaaabaaaaaaaaaanmaaaaaaaa
aaaaabbiaaacaaagaaaeaaaaaaaaaapmaaaaaaaaaaaaabcgaaacaaakaaaeaaaa
aaaaaapmaaaaaaaaaaaaabdeaaacaaaaaaabaaaaaaaaabemaaaaaaaaaaaaabfm
aaacaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabhbaaacaaacaaaeaaaaaaaaaapm
aaaaaaaaaaaaabieaaacaaaoaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgiheengbhehcgjhi
daaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabpiaadbaaahaaaaaaaaaaaaaaaa
aaaadeieaaaaaaabaaaaaaaeaaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaah
aacafaaiaaaapafaaaachbfbaaafhcfcaaaihdfdaaaaaacdaaaabaceaaaaaabn
aaaaaaboaaaababpaaaaaacaaaaaaacbaaaabaccaaaabacipaffeaafaaaabcaa
mcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabj
gabpbcaabcaaaaaaaaaaeacfaaaaccaaaaaaaaaaafpigaaaaaaaagiiaaaaaaaa
afpifaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmi
aaaaaaaamiapaaaaaabliiaakbagafaamiapaaaaaamgiiaaklagaeaamiapaaaa
aalbdejeklagadaamiapiadoaagmaadeklagacaamiahaaaaaaleblaacbanabaa
miahaaadaamamgmaalamaaanmiahaaadaalelbleclalaaadmiahaaaeaalogfaa
obacafaamiahaaahaamamgleclamabaamiapaaaaaabliiaakbagajaamiapaaaa
aamgiiaaklagaiaamiahaaahaalelbleclalabahmiahaaaeabgflomaolacafae
miahaaadaamagmleclakaaadmiahaaadabmablmakladaoagmiahaaaeaamablaa
obaeafaamiahaaahaamagmleclakabahmiapaaaaaalbdejeklagahaamiapaaaa
aagmejhkklagagaamiahaaagabmablmaklahaoagmiabiaabaaloloaapaagafaa
miaciaabaaloloaapaaeagaamiaeiaabaaloloaapaagacaamiabiaacaaloloaa
paadafaamiaciaacaaloloaapaaeadaamiaeiaacaaloloaapaadacaamiadiaaa
aalalabkilabbdbdmiamiaaaaakmkmagilabbebemiahaaabaalbleaakbaabcaa
miahaaabaamgmaleklaabbabmiahaaaaaagmleleklaabaabmiahiaadaablmale
klaaapaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Matrix 268 [_LightMatrix0]
Vector 464 [_MainTex_ST]
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 33 instructions using 5 registers
[Configuration]
8
0000002141050500
[Microcode]
528
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00019c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01d0700d8106c0c360403ffc
00001c6c01d0600d8106c0c360405ffc00001c6c01d0500d8106c0c360409ffc
00001c6c01d0400d8106c0c360411ffc00021c6c01d0a00d8286c0c360405ffc
00021c6c01d0900d8286c0c360409ffc00021c6c01d0800d8286c0c360411ffc
00009c6c0190a00c0686c0c360405ffc00009c6c0190900c0686c0c360409ffc
00009c6c0190800c0686c0c360411ffc00019c6c00800243011842436041dffc
00011c6c010002308121826301a1dffc401f9c6c01d0e00d8086c0c360405fa8
401f9c6c01d0d00d8086c0c360409fa8401f9c6c01d0c00d8086c0c360411fa8
00001c6c011d100c08bfc0e30041dffc00009c6c011d100c02bfc0e30041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
00011c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106014360405fa4
401f9c6c01400e0c0106014360411fa4401f9c6c0140000c0086024360409fa0
401f9c6c0140000c0486014360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedgomfbnegpddfhmcimimafalhinaolgnhabaaaaaaceahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefciiafaaaaeaaaabaa
gcabaaaafjaaaaaeegiocaaaaaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaalaaaaaa
kgiocaaaaaaaaaaaalaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaa
aaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
pgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaa
baaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
adaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaa
aaaaaaaaaeaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaa
agaabaaaaaaaaaaaegacbaaaabaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
aaaaaaaaafaaaaaakgakbaaaaaaaaaaaegacbaaaabaaaaaadcaaaaakhccabaaa
aeaaaaaaegiccaaaaaaaaaaaagaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
doaaaaab"
}

SubProgram "gles " {
Keywords { "POINT" }
"!!GLES


#ifdef VERTEX

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_4;
  tmpvar_4 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_7;
  mediump vec3 tmpvar_8;
  tmpvar_8 = normalize(xlv_TEXCOORD2);
  highp float tmpvar_9;
  tmpvar_9 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_LightTexture0, vec2(tmpvar_9));
  mediump vec3 lightDir_11;
  lightDir_11 = lightDir_2;
  mediump float atten_12;
  atten_12 = tmpvar_10.w;
  mediump vec4 c_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (tmpvar_4, lightDir_11));
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (tmpvar_4, normalize((lightDir_11 + tmpvar_8))));
  nh_14 = tmpvar_16;
  mediump float tmpvar_17;
  tmpvar_17 = ((1.0 - dot (normalize(tmpvar_8), tmpvar_4)) + dot (normalize(lightDir_11), tmpvar_4));
  highp vec3 tmpvar_18;
  tmpvar_18 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_15) + ((((_LightColor0.xyz * pow (nh_14, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * (atten_12 * 2.0));
  c_13.xyz = tmpvar_18;
  highp vec3 tmpvar_19;
  tmpvar_19 = (c_13.xyz + (c_13.xyz * (pow (tmpvar_17, _RimPower) * _RimColor.xyz)));
  c_13.xyz = tmpvar_19;
  c_13.w = tmpvar_6;
  c_1.xyz = c_13.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "POINT" }
"!!GLES


#ifdef VERTEX

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 normal_4;
  normal_4.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_4.z = sqrt((1.0 - clamp (dot (normal_4.xy, normal_4.xy), 0.0, 1.0)));
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_7;
  mediump vec3 tmpvar_8;
  tmpvar_8 = normalize(xlv_TEXCOORD2);
  highp float tmpvar_9;
  tmpvar_9 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_LightTexture0, vec2(tmpvar_9));
  mediump vec3 lightDir_11;
  lightDir_11 = lightDir_2;
  mediump float atten_12;
  atten_12 = tmpvar_10.w;
  mediump vec4 c_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (normal_4, lightDir_11));
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (normal_4, normalize((lightDir_11 + tmpvar_8))));
  nh_14 = tmpvar_16;
  mediump float tmpvar_17;
  tmpvar_17 = ((1.0 - dot (normalize(tmpvar_8), normal_4)) + dot (normalize(lightDir_11), normal_4));
  highp vec3 tmpvar_18;
  tmpvar_18 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_15) + ((((_LightColor0.xyz * pow (nh_14, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * (atten_12 * 2.0));
  c_13.xyz = tmpvar_18;
  highp vec3 tmpvar_19;
  tmpvar_19 = (c_13.xyz + (c_13.xyz * (pow (tmpvar_17, _RimPower) * _RimColor.xyz)));
  c_13.xyz = tmpvar_19;
  c_13.w = tmpvar_6;
  c_1.xyz = c_13.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
Vector 20 [_BumpMap_ST]
"agal_vs
c21 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacbfaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c21.x
aaaaaaaaaaaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c16
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaacaaahacabaaaakeacaaaaaabcaaaappabaaaaaa mul r2.xyz, r1.xyzz, c18.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c17, r0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c17, r1
adaaaaaaabaaahacaeaaaakeacaaaaaabcaaaappabaaaaaa mul r1.xyz, r4.xyzz, c18.w
acaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r1.xyzz, a0
bcaaaaaaabaaacaeaaaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r0.xyzz, r2.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v1.z, a1, r0.xyzz
bcaaaaaaabaaabaeaaaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r0.xyzz, a5
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
bdaaaaaaadaaaeaeaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 v3.z, r0, c14
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaaaaamacadaaaaeeaaaaaaaabeaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c20.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaabeaaaaoeabaaaaaa add v0.zw, r0.wwzw, c20
adaaaaaaaaaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r0.xy, a3, c19
abaaaaaaaaaaadaeaaaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r0.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedoooijienlngbiijahoolihhilkbgbnilabaaaaaahiakaaaaaeaaaaaa
daaaaaaaiaadaaaabaajaaaaniajaaaaebgpgodjeiadaaaaeiadaaaaaaacpopp
niacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaakaaacaaafaaaaaaaaaaabaaaeaaabaaahaaaaaaaaaa
acaaaaaaabaaaiaaaaaaaaaaadaaaaaaaeaaajaaaaaaaaaaadaaamaaajaaanaa
aaaaaaaaaaaaaaaaaaacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaaeaaaaaeaaaaamoaadaaeejaagaaeeka
agaaoekaabaaaaacaaaaapiaaiaaoekaafaaaaadabaaahiaaaaaffiabcaaoeka
aeaaaaaeabaaahiabbaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaakkiaabaaoeiaaeaaaaaeaaaaahiabeaaoekaaaaappiaaaaaoeiaaeaaaaae
aaaaahiaaaaaoeiabfaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaaaaaoeia
abaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaae
abaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappja
aiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeia
abaaaaacaaaaahiaahaaoekaafaaaaadacaaahiaaaaaffiabcaaoekaaeaaaaae
aaaaaliabbaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiabdaaoekaaaaakkia
aaaapeiaacaaaaadaaaaahiaaaaaoeiabeaaoekaaeaaaaaeaaaaahiaaaaaoeia
bfaappkaaaaaoejbaiaaaaadacaaaboaabaaoejaaaaaoeiaaiaaaaadacaaacoa
abaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaapia
aaaaffjaaoaaoekaaeaaaaaeaaaaapiaanaaoekaaaaaaajaaaaaoeiaaeaaaaae
aaaaapiaapaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiabaaaoekaaaaappja
aaaaoeiaafaaaaadabaaahiaaaaaffiaacaaoekaaeaaaaaeabaaahiaabaaoeka
aaaaaaiaabaaoeiaaeaaaaaeaaaaahiaadaaoekaaaaakkiaabaaoeiaaeaaaaae
adaaahoaaeaaoekaaaaappiaaaaaoeiaafaaaaadaaaaapiaaaaaffjaakaaoeka
aeaaaaaeaaaaapiaajaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaalaaoeka
aaaakkjaaaaaoeiaaeaaaaaeaaaaapiaamaaoekaaaaappjaaaaaoeiaaeaaaaae
aaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaa
fdeieefciiafaaaaeaaaabaagcabaaaafjaaaaaeegiocaaaaaaaaaaaamaaaaaa
fjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaa
fjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaad
pcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaa
ghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaad
hccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaa
giaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaa
ogikcaaaaaaaaaaaakaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaa
agiecaaaaaaaaaaaalaaaaaakgiocaaaaaaaaaaaalaaaaaadiaaaaahhcaabaaa
aaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaaaaaaaaaaeaaaaaadcaaaaakhcaabaaaabaaaaaa
egiccaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaakhccabaaaaeaaaaaaegiccaaaaaaaaaaaagaaaaaapgapbaaa
aaaaaaaaegacbaaaaaaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "gles3 " {
Keywords { "POINT" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 402
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 417
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
    highp vec3 _LightCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform highp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform sampler2D _LightTexture0;
uniform highp mat4 _LightMatrix0;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
#line 388
uniform highp float _RimPower;
#line 409
#line 426
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return ((objSpaceLightPos.xyz * unity_Scale.w) - v.xyz);
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 428
v2f_surf vert_surf( in appdata_full v ) {
    #line 430
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 434
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    o.lightDir = lightDir;
    #line 438
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    o._LightCoord = (_LightMatrix0 * (_Object2World * v.vertex)).xyz;
    #line 442
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out mediump vec3 xlv_TEXCOORD1;
out mediump vec3 xlv_TEXCOORD2;
out highp vec3 xlv_TEXCOORD3;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD2 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD3 = vec3(xl_retval._LightCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 402
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 417
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
    highp vec3 _LightCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform highp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform sampler2D _LightTexture0;
uniform highp mat4 _LightMatrix0;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
#line 388
uniform highp float _RimPower;
#line 409
#line 426
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 389
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    mediump vec3 h = normalize((lightDir + viewDir));
    #line 392
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    #line 396
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    c.w = s.Alpha;
    #line 400
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 409
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 413
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 444
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 446
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    SurfaceOutput o;
    #line 450
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    o.Alpha = 0.0;
    #line 454
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp vec3 lightDir = normalize(IN.lightDir);
    lowp vec4 c = LightingSimpleSpecular( o, lightDir, normalize(IN.viewDir), (texture( _LightTexture0, vec2( dot( IN._LightCoord, IN._LightCoord))).w * 1.0));
    #line 458
    c.w = 0.0;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in mediump vec3 xlv_TEXCOORD1;
in mediump vec3 xlv_TEXCOORD2;
in highp vec3 xlv_TEXCOORD3;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD1);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD2);
    xlt_IN._LightCoord = vec3(xlv_TEXCOORD3);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 9 [_WorldSpaceCameraPos]
Vector 10 [_WorldSpaceLightPos0]
Matrix 5 [_World2Object]
Vector 11 [unity_Scale]
Vector 12 [_MainTex_ST]
Vector 13 [_BumpMap_ST]
"!!ARBvp1.0
# 26 ALU
PARAM c[14] = { { 1 },
		state.matrix.mvp,
		program.local[5..13] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[9];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[7];
DP4 R2.y, R1, c[6];
DP4 R2.x, R1, c[5];
MAD R2.xyz, R2, c[11].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[10];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[7];
DP4 R3.y, R0, c[6];
DP4 R3.x, R0, c[5];
DP3 result.texcoord[1].y, R3, R1;
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[1].z, vertex.normal, R3;
DP3 result.texcoord[1].x, R3, vertex.attrib[14];
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[13].xyxy, c[13];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[12], c[12].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 26 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 8 [_WorldSpaceCameraPos]
Vector 9 [_WorldSpaceLightPos0]
Matrix 4 [_World2Object]
Vector 10 [unity_Scale]
Vector 11 [_MainTex_ST]
Vector 12 [_BumpMap_ST]
"vs_2_0
; 29 ALU
def c13, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c13.x
mov r0.xyz, c8
dp4 r1.z, r0, c6
dp4 r1.y, r0, c5
dp4 r1.x, r0, c4
mad r3.xyz, r1, c10.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c6
dp4 r4.z, c9, r0
mov r0, c5
mov r1, c4
dp4 r4.y, c9, r0
dp4 r4.x, c9, r1
dp3 oT1.y, r4, r2
dp3 oT2.y, r2, r3
dp3 oT1.z, v2, r4
dp3 oT1.x, r4, v1
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
mad oT0.zw, v3.xyxy, c12.xyxy, c12
mad oT0.xy, v3, c11, c11.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 12 [_BumpMap_ST]
Vector 11 [_MainTex_ST]
Matrix 6 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 10 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 30.67 (23 instructions), vertex: 32, texture: 0,
//   sequencer: 16,  7 GPRs, 27 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaabomaaaaabiaaaaaaaaaaaaaaaceaaaaaaaaaaaaabiiaaaaaaaa
aaaaaaaaaaaaabgaaaaaaabmaaaaabfdpppoadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabemaaaaaakiaaacaaamaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaal
aaabaaaaaaaaaaleaaaaaaaaaaaaaanaaaacaaagaaaeaaaaaaaaaaoaaaaaaaaa
aaaaaapaaaacaaaaaaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaabaaabaaaa
aaaaaaleaaaaaaaaaaaaabcnaaacaaacaaaeaaaaaaaaaaoaaaaaaaaaaaaaabea
aaacaaakaaabaaaaaaaaaaleaaaaaaaafpechfgnhaengbhafpfdfeaaaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpfhgphcgmgedcep
gcgkgfgdheaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgefdha
gbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaaaaaaaaaa
fpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegffpgngbhe
hcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodc
dadddfddcodaaaklaaaaaaaaaaaaabiaaacbaaagaaaaaaaaaaaaaaaaaaaacigd
aaaaaaabaaaaaaaeaaaaaaaiaaaaacjaaabaaaaeaaaagaafaaaadaagaadafaah
aaaapafaaaachbfbaaafhcfcaaaaaabnaaaababoaaaaaabhaaaaaabiaaaababj
aaaaaabkaaaaaablaaaababmpaffeaaeaaaabcaamcaaaaaaaaaaeaaiaaaabcaa
meaaaaaaaaaagaamgabcbcaabcaaaaaaaaaagabibabobcaaccaaaaaaafpicaaa
aaaaagiiaaaaaaaaafpieaaaaaaaagiiaaaaaaaaafpibaaaaaaaaoiiaaaaaaaa
afpiaaaaaaaaapmiaaaaaaaamiapaaadaabliiaakbacafaamiapaaadaamgiiaa
klacaeadmiapaaadaalbdejeklacadadmiapiadoaagmaadeklacacadmiahaaaf
aaleblaacbajabaamiahaaadaamamgmaalaiaaajmiahaaagaalelbleclahaaad
miahaaadaalogfaaobabaeaamiahaaafaamamgleclaiabafmiahaaafaalelble
clahabafmiahaaadabgflomaolabaeadmiahaaagaamagmleclagaaagmiahaaac
abmablmaklagakacmiahaaadaamablaaobadaeaamiahaaafaamagmleclagabaf
miabiaabaaloloaapaafaeaamiaciaabaaloloaapaadafaamiaeiaabaaloloaa
paafabaamiabiaacaaloloaapaacaeaamiaciaacaaloloaapaadacaamiaeiaac
aaloloaapaacabaamiadiaaaaalalabkilaaalalmiamiaaaaakmkmagilaaamam
aaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_World2Object]
Vector 465 [unity_Scale]
Vector 464 [_MainTex_ST]
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 25 instructions using 4 registers
[Configuration]
8
0000001941050400
[Microcode]
400
00001c6c005d200d8186c0836041fffc00009c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000019c6c01d0600d8086c0c360405ffc
00019c6c01d0500d8086c0c360409ffc00019c6c01d0400d8086c0c360411ffc
00001c6c0190600c0486c0c360405ffc00001c6c0190500c0486c0c360409ffc
00001c6c0190400c0486c0c360411ffc00011c6c00800243011841436041dffc
00009c6c01000230812181630121dffc00001c6c011d100c00bfc0e30041dffc
401f9c6c0140020c0106034360405fa0401f9c6c01400e0c0686008360411fa0
00009c6c00800e0c02bfc0836041dffc401f9c6c0140020c0106004360405fa4
401f9c6c01400e0c0106004360411fa4401f9c6c0140000c0686014360409fa0
401f9c6c0140000c0286004360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 128 used size, 8 vars
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 25 instructions, 2 temp regs, 0 temp arrays:
// ALU 12 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecededgefjbkkifamildajilkfiealcbefhoabaaaaaakeafaaaaadaaaaaa
cmaaaaaapeaaaaaahmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaa
aiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaheaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
caaeaaaaeaaaabaaaiabaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadhccabaaaadaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaa
aaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaa
pgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaa
adaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaadcaaaaal
mccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaahaaaaaakgiocaaa
aaaaaaaaahaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaa
acaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaa
egacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaa
pgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
aaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
doaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" }
"!!GLES


#ifdef VERTEX

varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 glstate_matrix_mvp;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
}



#endif
#ifdef FRAGMENT

varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_4;
  tmpvar_4 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  lightDir_2 = xlv_TEXCOORD1;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD2);
  mediump vec3 lightDir_8;
  lightDir_8 = lightDir_2;
  mediump vec4 c_9;
  highp float nh_10;
  mediump float tmpvar_11;
  tmpvar_11 = max (0.0, dot (tmpvar_4, lightDir_8));
  mediump float tmpvar_12;
  tmpvar_12 = max (0.0, dot (tmpvar_4, normalize((lightDir_8 + tmpvar_7))));
  nh_10 = tmpvar_12;
  mediump float tmpvar_13;
  tmpvar_13 = ((1.0 - dot (normalize(tmpvar_7), tmpvar_4)) + dot (normalize(lightDir_8), tmpvar_4));
  highp vec3 tmpvar_14;
  tmpvar_14 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_11) + ((((_LightColor0.xyz * pow (nh_10, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * 2.0);
  c_9.xyz = tmpvar_14;
  highp vec3 tmpvar_15;
  tmpvar_15 = (c_9.xyz + (c_9.xyz * (pow (tmpvar_13, _RimPower) * _RimColor.xyz)));
  c_9.xyz = tmpvar_15;
  c_9.w = tmpvar_6;
  c_1.xyz = c_9.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" }
"!!GLES


#ifdef VERTEX

varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 glstate_matrix_mvp;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
}



#endif
#ifdef FRAGMENT

varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 normal_4;
  normal_4.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_4.z = sqrt((1.0 - clamp (dot (normal_4.xy, normal_4.xy), 0.0, 1.0)));
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  lightDir_2 = xlv_TEXCOORD1;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD2);
  mediump vec3 lightDir_8;
  lightDir_8 = lightDir_2;
  mediump vec4 c_9;
  highp float nh_10;
  mediump float tmpvar_11;
  tmpvar_11 = max (0.0, dot (normal_4, lightDir_8));
  mediump float tmpvar_12;
  tmpvar_12 = max (0.0, dot (normal_4, normalize((lightDir_8 + tmpvar_7))));
  nh_10 = tmpvar_12;
  mediump float tmpvar_13;
  tmpvar_13 = ((1.0 - dot (normalize(tmpvar_7), normal_4)) + dot (normalize(lightDir_8), normal_4));
  highp vec3 tmpvar_14;
  tmpvar_14 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_11) + ((((_LightColor0.xyz * pow (nh_10, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * 2.0);
  c_9.xyz = tmpvar_14;
  highp vec3 tmpvar_15;
  tmpvar_15 = (c_9.xyz + (c_9.xyz * (pow (tmpvar_13, _RimPower) * _RimColor.xyz)));
  c_9.xyz = tmpvar_15;
  c_9.w = tmpvar_6;
  c_1.xyz = c_9.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 8 [_WorldSpaceCameraPos]
Vector 9 [_WorldSpaceLightPos0]
Matrix 4 [_World2Object]
Vector 10 [unity_Scale]
Vector 11 [_MainTex_ST]
Vector 12 [_BumpMap_ST]
"agal_vs
c13 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacanaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c13.x
aaaaaaaaaaaaahacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c8
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaagaaaaoeabaaaaaa dp4 r1.z, r0, c6
bdaaaaaaabaaacacaaaaaaoeacaaaaaaafaaaaoeabaaaaaa dp4 r1.y, r0, c5
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaeaaaaoeabaaaaaa dp4 r1.x, r0, c4
adaaaaaaacaaahacabaaaakeacaaaaaaakaaaappabaaaaaa mul r2.xyz, r1.xyzz, c10.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacagaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c6
bdaaaaaaaeaaaeacajaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c9, r0
aaaaaaaaaaaaapacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c5
aaaaaaaaabaaapacaeaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c4
bdaaaaaaaeaaacacajaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c9, r0
bdaaaaaaaeaaabacajaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c9, r1
bcaaaaaaabaaacaeaeaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r4.xyzz, r2.xyzz
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v1.z, a1, r4.xyzz
bcaaaaaaabaaabaeaeaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r4.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
adaaaaaaaaaaamacadaaaaeeaaaaaaaaamaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c12.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaaamaaaaoeabaaaaaa add v0.zw, r0.wwzw, c12
adaaaaaaaaaaadacadaaaaoeaaaaaaaaalaaaaoeabaaaaaa mul r0.xy, a3, c11
abaaaaaaaaaaadaeaaaaaafeacaaaaaaalaaaaooabaaaaaa add v0.xy, r0.xyyy, c11.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 128 used size, 8 vars
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 25 instructions, 2 temp regs, 0 temp arrays:
// ALU 12 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedhidelnhidhhlcckofdohhjdbinciopfkabaaaaaaeaaiaaaaaeaaaaaa
daaaaaaamiacaaaapaagaaaaliahaaaaebgpgodjjaacaaaajaacaaaaaaacpopp
cmacaaaageaaaaaaafaaceaaaaaagaaaaaaagaaaaaaaceaaabaagaaaaaaaagaa
acaaabaaaaaaaaaaabaaaeaaabaaadaaaaaaaaaaacaaaaaaabaaaeaaaaaaaaaa
adaaaaaaaeaaafaaaaaaaaaaadaabaaaafaaajaaaaaaaaaaaaaaaaaaaaacpopp
bpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabiaabaaapjabpaaaaacafaaacia
acaaapjabpaaaaacafaaadiaadaaapjaaeaaaaaeaaaaadoaadaaoejaabaaoeka
abaaookaaeaaaaaeaaaaamoaadaaeejaacaaeekaacaaoekaabaaaaacaaaaapia
aeaaoekaafaaaaadabaaahiaaaaaffiaakaaoekaaeaaaaaeabaaahiaajaaoeka
aaaaaaiaabaaoeiaaeaaaaaeaaaaahiaalaaoekaaaaakkiaabaaoeiaaeaaaaae
aaaaahiaamaaoekaaaaappiaaaaaoeiaaiaaaaadabaaaboaabaaoejaaaaaoeia
abaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaae
abaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappja
aiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeia
abaaaaacaaaaahiaadaaoekaafaaaaadacaaahiaaaaaffiaakaaoekaaeaaaaae
aaaaaliaajaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiaalaaoekaaaaakkia
aaaapeiaacaaaaadaaaaahiaaaaaoeiaamaaoekaaeaaaaaeaaaaahiaaaaaoeia
anaappkaaaaaoejbaiaaaaadacaaaboaabaaoejaaaaaoeiaaiaaaaadacaaacoa
abaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaapia
aaaaffjaagaaoekaaeaaaaaeaaaaapiaafaaoekaaaaaaajaaaaaoeiaaeaaaaae
aaaaapiaahaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaaiaaoekaaaaappja
aaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaamma
aaaaoeiappppaaaafdeieefccaaeaaaaeaaaabaaaiabaaaafjaaaaaeegiocaaa
aaaaaaaaaiaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaa
acaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaa
aaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaad
dcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaa
abaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagiaaaaac
acaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
abaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
acaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaa
egiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaal
dccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaa
aaaaaaaaagaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaa
aaaaaaaaahaaaaaakgiocaaaaaaaaaaaahaaaaaadiaaaaahhcaabaaaaaaaaaaa
jgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaa
acaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
cccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaa
abaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaa
aeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaa
adaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaa
egbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaa
acaaaaaaegacbaaaabaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaa
aeaaaaaaaiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
heaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaheaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 400
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 415
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 384
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 407
#line 423
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 440
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 425
v2f_surf vert_surf( in appdata_full v ) {
    #line 427
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 431
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    o.lightDir = lightDir;
    #line 435
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out mediump vec3 xlv_TEXCOORD1;
out mediump vec3 xlv_TEXCOORD2;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD2 = vec3(xl_retval.viewDir);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 400
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 415
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform highp float _Shininess;
uniform sampler2D _MainTex;
#line 384
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 407
#line 423
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 440
#line 387
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    #line 389
    mediump vec3 h = normalize((lightDir + viewDir));
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    #line 393
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    #line 397
    c.w = s.Alpha;
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 407
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 411
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 440
lowp vec4 frag_surf( in v2f_surf IN ) {
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    #line 444
    surfIN.uv_BumpMap = IN.pack0.zw;
    SurfaceOutput o;
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    #line 448
    o.Specular = 0.0;
    o.Alpha = 0.0;
    o.Gloss = 0.0;
    surf( surfIN, o);
    #line 452
    lowp vec3 lightDir = IN.lightDir;
    lowp vec4 c = LightingSimpleSpecular( o, lightDir, normalize(IN.viewDir), 1.0);
    c.w = 0.0;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in mediump vec3 xlv_TEXCOORD1;
in mediump vec3 xlv_TEXCOORD2;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD1);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD2);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "opengl " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_WorldSpaceLightPos0]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 19 [unity_Scale]
Matrix 13 [_LightMatrix0]
Vector 20 [_MainTex_ST]
Vector 21 [_BumpMap_ST]
"!!ARBvp1.0
# 35 ALU
PARAM c[22] = { { 1 },
		state.matrix.mvp,
		program.local[5..21] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[18];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.x, R0, c[9];
DP4 R3.y, R0, c[10];
MAD R0.xyz, R3, c[19].w, -vertex.position;
DP4 R0.w, vertex.position, c[8];
DP3 result.texcoord[1].y, R0, R1;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
DP4 result.texcoord[3].w, R0, c[16];
DP4 result.texcoord[3].z, R0, c[15];
DP4 result.texcoord[3].y, R0, c[14];
DP4 result.texcoord[3].x, R0, c[13];
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[21].xyxy, c[21];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 35 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
Vector 20 [_BumpMap_ST]
"vs_2_0
; 38 ALU
def c21, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c21.x
mov r0.xyz, c16
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c18.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c17, r0
mov r0, c9
dp4 r4.y, c17, r0
mov r1, c8
dp4 r4.x, c17, r1
mad r0.xyz, r4, c18.w, -v0
dp4 r0.w, v0, c7
dp3 oT1.y, r0, r2
dp3 oT1.z, v2, r0
dp3 oT1.x, r0, v1
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 oT2.y, r2, r3
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
dp4 oT3.w, r0, c15
dp4 oT3.z, r0, c14
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.zw, v3.xyxy, c20.xyxy, c20
mad oT0.xy, v3, c19, c19.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 20 [_BumpMap_ST]
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 42.67 (32 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacdiaaaaabpiaaaaaaaaaaaaaaceaaaaaaaaaaaaabmmaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaabeaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaap
aaaeaaaaaaaaaapmaaaaaaaaaaaaabamaaacaabdaaabaaaaaaaaaanmaaaaaaaa
aaaaabbiaaacaaagaaaeaaaaaaaaaapmaaaaaaaaaaaaabcgaaacaaakaaaeaaaa
aaaaaapmaaaaaaaaaaaaabdeaaacaaaaaaabaaaaaaaaabemaaaaaaaaaaaaabfm
aaacaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabhbaaacaaacaaaeaaaaaaaaaapm
aaaaaaaaaaaaabieaaacaaaoaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgiheengbhehcgjhi
daaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabpiaadbaaahaaaaaaaaaaaaaaaa
aaaadiieaaaaaaabaaaaaaaeaaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaah
aadafaaiaaaapafaaaachbfbaaafhcfcaaaipdfdaaaaaacdaaaabaceaaaaaabn
aaaaaaboaaaababpaaaaaacaaaaaaacbaaaabaccaaaabacipaffeaafaaaabcaa
mcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabj
gabpbcaabcaaaaaaaaaaeacfaaaaccaaaaaaaaaaafpigaaaaaaaagiiaaaaaaaa
afpifaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpiaaaaaaaaapmi
aaaaaaaamiapaaabaabliiaakbagafaamiapaaabaamgiiaaklagaeabmiapaaab
aalbdejeklagadabmiapiadoaagmaadeklagacabmiahaaabaaleblaacbanabaa
miahaaadaamamgmaalamaaanmiahaaadaalelbleclalaaadmiahaaaeaalogfaa
obacafaamiahaaahaamamgleclamababmiapaaabaabliiaakbagajaamiapaaab
aamgiiaaklagaiabmiahaaahaalelbleclalabahmiahaaaeabgflomaolacafae
miahaaadaamagmleclakaaadmiahaaadabmablmakladaoagmiahaaaeaamablaa
obaeafaamiahaaahaamagmleclakabahmiapaaabaalbdejeklagahabmiapaaab
aagmnajeklagagabmiahaaagabmablmaklahaoagmiabiaabaaloloaapaagafaa
miaciaabaaloloaapaaeagaamiaeiaabaaloloaapaagacaamiabiaacaaloloaa
paadafaamiaciaacaaloloaapaaeadaamiaeiaacaaloloaapaadacaamiadiaaa
aalalabkilaabdbdmiamiaaaaakmkmagilaabebemiapaaaaaamgiiaakbabbcaa
miapaaaaaabliiaaklabbbaamiapaaaaaalbdejeklabbaaamiapiaadaagmaade
klabapaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "SPOT" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Matrix 268 [_LightMatrix0]
Vector 464 [_MainTex_ST]
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 34 instructions using 5 registers
[Configuration]
8
0000002241050500
[Microcode]
544
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00019c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01d0700d8106c0c360403ffc
00001c6c01d0600d8106c0c360405ffc00001c6c01d0500d8106c0c360409ffc
00001c6c01d0400d8106c0c360411ffc00021c6c01d0a00d8286c0c360405ffc
00021c6c01d0900d8286c0c360409ffc00021c6c01d0800d8286c0c360411ffc
00009c6c0190a00c0686c0c360405ffc00009c6c0190900c0686c0c360409ffc
00009c6c0190800c0686c0c360411ffc00019c6c00800243011842436041dffc
00011c6c010002308121826301a1dffc401f9c6c01d0f00d8086c0c360403fa8
401f9c6c01d0e00d8086c0c360405fa8401f9c6c01d0d00d8086c0c360409fa8
401f9c6c01d0c00d8086c0c360411fa800001c6c011d100c08bfc0e30041dffc
00009c6c011d100c02bfc0e30041dffc401f9c6c0140020c0106004360405fa0
401f9c6c01400e0c0086008360411fa000011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106014360405fa4401f9c6c01400e0c0106014360411fa4
401f9c6c0140000c0086024360409fa0401f9c6c0140000c0486014360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedeepalmgddjbhgnfdapphknkckbdllhgdabaaaaaaceahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefciiafaaaaeaaaabaa
gcabaaaafjaaaaaeegiocaaaaaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaalaaaaaa
kgiocaaaaaaaaaaaalaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaa
aaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
pgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaa
baaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
adaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadiaaaaaipcaabaaaabaaaaaafgafbaaaaaaaaaaaegiocaaa
aaaaaaaaaeaaaaaadcaaaaakpcaabaaaabaaaaaaegiocaaaaaaaaaaaadaaaaaa
agaabaaaaaaaaaaaegaobaaaabaaaaaadcaaaaakpcaabaaaabaaaaaaegiocaaa
aaaaaaaaafaaaaaakgakbaaaaaaaaaaaegaobaaaabaaaaaadcaaaaakpccabaaa
aeaaaaaaegiocaaaaaaaaaaaagaaaaaapgapbaaaaaaaaaaaegaobaaaabaaaaaa
doaaaaab"
}

SubProgram "gles " {
Keywords { "SPOT" }
"!!GLES


#ifdef VERTEX

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _LightTextureB0;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_4;
  tmpvar_4 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_7;
  mediump vec3 tmpvar_8;
  tmpvar_8 = normalize(xlv_TEXCOORD2);
  lowp vec4 tmpvar_9;
  highp vec2 P_10;
  P_10 = ((xlv_TEXCOORD3.xy / xlv_TEXCOORD3.w) + 0.5);
  tmpvar_9 = texture2D (_LightTexture0, P_10);
  highp float tmpvar_11;
  tmpvar_11 = dot (xlv_TEXCOORD3.xyz, xlv_TEXCOORD3.xyz);
  lowp vec4 tmpvar_12;
  tmpvar_12 = texture2D (_LightTextureB0, vec2(tmpvar_11));
  mediump vec3 lightDir_13;
  lightDir_13 = lightDir_2;
  mediump float atten_14;
  atten_14 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_9.w) * tmpvar_12.w);
  mediump vec4 c_15;
  highp float nh_16;
  mediump float tmpvar_17;
  tmpvar_17 = max (0.0, dot (tmpvar_4, lightDir_13));
  mediump float tmpvar_18;
  tmpvar_18 = max (0.0, dot (tmpvar_4, normalize((lightDir_13 + tmpvar_8))));
  nh_16 = tmpvar_18;
  mediump float tmpvar_19;
  tmpvar_19 = ((1.0 - dot (normalize(tmpvar_8), tmpvar_4)) + dot (normalize(lightDir_13), tmpvar_4));
  highp vec3 tmpvar_20;
  tmpvar_20 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_17) + ((((_LightColor0.xyz * pow (nh_16, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * (atten_14 * 2.0));
  c_15.xyz = tmpvar_20;
  highp vec3 tmpvar_21;
  tmpvar_21 = (c_15.xyz + (c_15.xyz * (pow (tmpvar_19, _RimPower) * _RimColor.xyz)));
  c_15.xyz = tmpvar_21;
  c_15.w = tmpvar_6;
  c_1.xyz = c_15.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "SPOT" }
"!!GLES


#ifdef VERTEX

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _LightTextureB0;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 normal_4;
  normal_4.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_4.z = sqrt((1.0 - clamp (dot (normal_4.xy, normal_4.xy), 0.0, 1.0)));
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_7;
  mediump vec3 tmpvar_8;
  tmpvar_8 = normalize(xlv_TEXCOORD2);
  lowp vec4 tmpvar_9;
  highp vec2 P_10;
  P_10 = ((xlv_TEXCOORD3.xy / xlv_TEXCOORD3.w) + 0.5);
  tmpvar_9 = texture2D (_LightTexture0, P_10);
  highp float tmpvar_11;
  tmpvar_11 = dot (xlv_TEXCOORD3.xyz, xlv_TEXCOORD3.xyz);
  lowp vec4 tmpvar_12;
  tmpvar_12 = texture2D (_LightTextureB0, vec2(tmpvar_11));
  mediump vec3 lightDir_13;
  lightDir_13 = lightDir_2;
  mediump float atten_14;
  atten_14 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_9.w) * tmpvar_12.w);
  mediump vec4 c_15;
  highp float nh_16;
  mediump float tmpvar_17;
  tmpvar_17 = max (0.0, dot (normal_4, lightDir_13));
  mediump float tmpvar_18;
  tmpvar_18 = max (0.0, dot (normal_4, normalize((lightDir_13 + tmpvar_8))));
  nh_16 = tmpvar_18;
  mediump float tmpvar_19;
  tmpvar_19 = ((1.0 - dot (normalize(tmpvar_8), normal_4)) + dot (normalize(lightDir_13), normal_4));
  highp vec3 tmpvar_20;
  tmpvar_20 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_17) + ((((_LightColor0.xyz * pow (nh_16, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * (atten_14 * 2.0));
  c_15.xyz = tmpvar_20;
  highp vec3 tmpvar_21;
  tmpvar_21 = (c_15.xyz + (c_15.xyz * (pow (tmpvar_19, _RimPower) * _RimColor.xyz)));
  c_15.xyz = tmpvar_21;
  c_15.w = tmpvar_6;
  c_1.xyz = c_15.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
Vector 20 [_BumpMap_ST]
"agal_vs
c21 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacbfaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c21.x
aaaaaaaaaaaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c16
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaacaaahacabaaaakeacaaaaaabcaaaappabaaaaaa mul r2.xyz, r1.xyzz, c18.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c17, r0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c17, r1
adaaaaaaabaaahacaeaaaakeacaaaaaabcaaaappabaaaaaa mul r1.xyz, r4.xyzz, c18.w
acaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r1.xyzz, a0
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bcaaaaaaabaaacaeaaaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r0.xyzz, r2.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v1.z, a1, r0.xyzz
bcaaaaaaabaaabaeaaaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r0.xyzz, a5
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
bdaaaaaaadaaaiaeaaaaaaoeacaaaaaaapaaaaoeabaaaaaa dp4 v3.w, r0, c15
bdaaaaaaadaaaeaeaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 v3.z, r0, c14
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaaaaamacadaaaaeeaaaaaaaabeaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c20.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaabeaaaaoeabaaaaaa add v0.zw, r0.wwzw, c20
adaaaaaaaaaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r0.xy, a3, c19
abaaaaaaaaaaadaeaaaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r0.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedgnhiabcolncfnabbjpdimnfmjdlpppdhabaaaaaahiakaaaaaeaaaaaa
daaaaaaaiaadaaaabaajaaaaniajaaaaebgpgodjeiadaaaaeiadaaaaaaacpopp
niacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaakaaacaaafaaaaaaaaaaabaaaeaaabaaahaaaaaaaaaa
acaaaaaaabaaaiaaaaaaaaaaadaaaaaaaeaaajaaaaaaaaaaadaaamaaajaaanaa
aaaaaaaaaaaaaaaaaaacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaaeaaaaaeaaaaamoaadaaeejaagaaeeka
agaaoekaabaaaaacaaaaapiaaiaaoekaafaaaaadabaaahiaaaaaffiabcaaoeka
aeaaaaaeabaaahiabbaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaakkiaabaaoeiaaeaaaaaeaaaaahiabeaaoekaaaaappiaaaaaoeiaaeaaaaae
aaaaahiaaaaaoeiabfaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaaaaaoeia
abaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaae
abaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappja
aiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeia
abaaaaacaaaaahiaahaaoekaafaaaaadacaaahiaaaaaffiabcaaoekaaeaaaaae
aaaaaliabbaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiabdaaoekaaaaakkia
aaaapeiaacaaaaadaaaaahiaaaaaoeiabeaaoekaaeaaaaaeaaaaahiaaaaaoeia
bfaappkaaaaaoejbaiaaaaadacaaaboaabaaoejaaaaaoeiaaiaaaaadacaaacoa
abaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaapia
aaaaffjaaoaaoekaaeaaaaaeaaaaapiaanaaoekaaaaaaajaaaaaoeiaaeaaaaae
aaaaapiaapaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiabaaaoekaaaaappja
aaaaoeiaafaaaaadabaaapiaaaaaffiaacaaoekaaeaaaaaeabaaapiaabaaoeka
aaaaaaiaabaaoeiaaeaaaaaeabaaapiaadaaoekaaaaakkiaabaaoeiaaeaaaaae
adaaapoaaeaaoekaaaaappiaabaaoeiaafaaaaadaaaaapiaaaaaffjaakaaoeka
aeaaaaaeaaaaapiaajaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaalaaoeka
aaaakkjaaaaaoeiaaeaaaaaeaaaaapiaamaaoekaaaaappjaaaaaoeiaaeaaaaae
aaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaa
fdeieefciiafaaaaeaaaabaagcabaaaafjaaaaaeegiocaaaaaaaaaaaamaaaaaa
fjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaa
fjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaad
pcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaa
ghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaad
hccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaa
giaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaa
ogikcaaaaaaaaaaaakaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaa
agiecaaaaaaaaaaaalaaaaaakgiocaaaaaaaaaaaalaaaaaadiaaaaahhcaabaaa
aaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaipcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiocaaaaaaaaaaaaeaaaaaadcaaaaakpcaabaaaabaaaaaa
egiocaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegaobaaaabaaaaaadcaaaaak
pcaabaaaabaaaaaaegiocaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegaobaaa
abaaaaaadcaaaaakpccabaaaaeaaaaaaegiocaaaaaaaaaaaagaaaaaapgapbaaa
aaaaaaaaegaobaaaabaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "gles3 " {
Keywords { "SPOT" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 411
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 426
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
    highp vec4 _LightCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform highp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform sampler2D _LightTexture0;
uniform highp mat4 _LightMatrix0;
#line 384
uniform sampler2D _LightTextureB0;
#line 389
#line 393
uniform highp float _Shininess;
uniform sampler2D _MainTex;
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
#line 397
uniform highp float _RimPower;
#line 418
#line 435
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return ((objSpaceLightPos.xyz * unity_Scale.w) - v.xyz);
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 437
v2f_surf vert_surf( in appdata_full v ) {
    #line 439
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 443
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    o.lightDir = lightDir;
    #line 447
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    o._LightCoord = (_LightMatrix0 * (_Object2World * v.vertex));
    #line 451
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out mediump vec3 xlv_TEXCOORD1;
out mediump vec3 xlv_TEXCOORD2;
out highp vec4 xlv_TEXCOORD3;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD2 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD3 = vec4(xl_retval._LightCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 411
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 426
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
    highp vec4 _LightCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform highp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform sampler2D _LightTexture0;
uniform highp mat4 _LightMatrix0;
#line 384
uniform sampler2D _LightTextureB0;
#line 389
#line 393
uniform highp float _Shininess;
uniform sampler2D _MainTex;
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
#line 397
uniform highp float _RimPower;
#line 418
#line 435
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 398
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    mediump vec3 h = normalize((lightDir + viewDir));
    #line 401
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    #line 405
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    c.w = s.Alpha;
    #line 409
    return c;
}
#line 389
lowp float UnitySpotAttenuate( in highp vec3 LightCoord ) {
    return texture( _LightTextureB0, vec2( dot( LightCoord, LightCoord))).w;
}
#line 385
lowp float UnitySpotCookie( in highp vec4 LightCoord ) {
    return texture( _LightTexture0, ((LightCoord.xy / LightCoord.w) + 0.5)).w;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 418
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 422
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 453
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 455
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    SurfaceOutput o;
    #line 459
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    o.Alpha = 0.0;
    #line 463
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp vec3 lightDir = normalize(IN.lightDir);
    lowp vec4 c = LightingSimpleSpecular( o, lightDir, normalize(IN.viewDir), (((float((IN._LightCoord.z > 0.0)) * UnitySpotCookie( IN._LightCoord)) * UnitySpotAttenuate( IN._LightCoord.xyz)) * 1.0));
    #line 467
    c.w = 0.0;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in mediump vec3 xlv_TEXCOORD1;
in mediump vec3 xlv_TEXCOORD2;
in highp vec4 xlv_TEXCOORD3;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD1);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD2);
    xlt_IN._LightCoord = vec4(xlv_TEXCOORD3);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "opengl " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_WorldSpaceLightPos0]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 19 [unity_Scale]
Matrix 13 [_LightMatrix0]
Vector 20 [_MainTex_ST]
Vector 21 [_BumpMap_ST]
"!!ARBvp1.0
# 34 ALU
PARAM c[22] = { { 1 },
		state.matrix.mvp,
		program.local[5..21] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[18];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.x, R0, c[9];
DP4 R3.y, R0, c[10];
MAD R0.xyz, R3, c[19].w, -vertex.position;
DP3 result.texcoord[1].y, R0, R1;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.w, vertex.position, c[8];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
DP4 result.texcoord[3].z, R0, c[15];
DP4 result.texcoord[3].y, R0, c[14];
DP4 result.texcoord[3].x, R0, c[13];
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[21].xyxy, c[21];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 34 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
Vector 20 [_BumpMap_ST]
"vs_2_0
; 37 ALU
def c21, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c21.x
mov r0.xyz, c16
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c18.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c17, r0
mov r0, c9
dp4 r4.y, c17, r0
mov r1, c8
dp4 r4.x, c17, r1
mad r0.xyz, r4, c18.w, -v0
dp3 oT1.y, r0, r2
dp3 oT1.z, v2, r0
dp3 oT1.x, r0, v1
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 oT2.y, r2, r3
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
dp4 oT3.z, r0, c14
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.zw, v3.xyxy, c20.xyxy, c20
mad oT0.xy, v3, c19, c19.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 20 [_BumpMap_ST]
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 42.67 (32 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacdiaaaaabpiaaaaaaaaaaaaaaceaaaaaaaaaaaaabmmaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaabeaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaap
aaaeaaaaaaaaaapmaaaaaaaaaaaaabamaaacaabdaaabaaaaaaaaaanmaaaaaaaa
aaaaabbiaaacaaagaaaeaaaaaaaaaapmaaaaaaaaaaaaabcgaaacaaakaaaeaaaa
aaaaaapmaaaaaaaaaaaaabdeaaacaaaaaaabaaaaaaaaabemaaaaaaaaaaaaabfm
aaacaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabhbaaacaaacaaaeaaaaaaaaaapm
aaaaaaaaaaaaabieaaacaaaoaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgiheengbhehcgjhi
daaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabpiaadbaaahaaaaaaaaaaaaaaaa
aaaadeieaaaaaaabaaaaaaaeaaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaah
aacafaaiaaaapafaaaachbfbaaafhcfcaaaihdfdaaaaaacdaaaabaceaaaaaabn
aaaaaaboaaaababpaaaaaacaaaaaaacbaaaabaccaaaabacipaffeaafaaaabcaa
mcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabj
gabpbcaabcaaaaaaaaaaeacfaaaaccaaaaaaaaaaafpigaaaaaaaagiiaaaaaaaa
afpifaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmi
aaaaaaaamiapaaaaaabliiaakbagafaamiapaaaaaamgiiaaklagaeaamiapaaaa
aalbdejeklagadaamiapiadoaagmaadeklagacaamiahaaaaaaleblaacbanabaa
miahaaadaamamgmaalamaaanmiahaaadaalelbleclalaaadmiahaaaeaalogfaa
obacafaamiahaaahaamamgleclamabaamiapaaaaaabliiaakbagajaamiapaaaa
aamgiiaaklagaiaamiahaaahaalelbleclalabahmiahaaaeabgflomaolacafae
miahaaadaamagmleclakaaadmiahaaadabmablmakladaoagmiahaaaeaamablaa
obaeafaamiahaaahaamagmleclakabahmiapaaaaaalbdejeklagahaamiapaaaa
aagmejhkklagagaamiahaaagabmablmaklahaoagmiabiaabaaloloaapaagafaa
miaciaabaaloloaapaaeagaamiaeiaabaaloloaapaagacaamiabiaacaaloloaa
paadafaamiaciaacaaloloaapaaeadaamiaeiaacaaloloaapaadacaamiadiaaa
aalalabkilabbdbdmiamiaaaaakmkmagilabbebemiahaaabaalbleaakbaabcaa
miahaaabaamgmaleklaabbabmiahaaaaaagmleleklaabaabmiahiaadaablmale
klaaapaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT_COOKIE" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Matrix 268 [_LightMatrix0]
Vector 464 [_MainTex_ST]
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 33 instructions using 5 registers
[Configuration]
8
0000002141050500
[Microcode]
528
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00019c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01d0700d8106c0c360403ffc
00001c6c01d0600d8106c0c360405ffc00001c6c01d0500d8106c0c360409ffc
00001c6c01d0400d8106c0c360411ffc00021c6c01d0a00d8286c0c360405ffc
00021c6c01d0900d8286c0c360409ffc00021c6c01d0800d8286c0c360411ffc
00009c6c0190a00c0686c0c360405ffc00009c6c0190900c0686c0c360409ffc
00009c6c0190800c0686c0c360411ffc00019c6c00800243011842436041dffc
00011c6c010002308121826301a1dffc401f9c6c01d0e00d8086c0c360405fa8
401f9c6c01d0d00d8086c0c360409fa8401f9c6c01d0c00d8086c0c360411fa8
00001c6c011d100c08bfc0e30041dffc00009c6c011d100c02bfc0e30041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
00011c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106014360405fa4
401f9c6c01400e0c0106014360411fa4401f9c6c0140000c0086024360409fa0
401f9c6c0140000c0486014360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedgomfbnegpddfhmcimimafalhinaolgnhabaaaaaaceahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefciiafaaaaeaaaabaa
gcabaaaafjaaaaaeegiocaaaaaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaalaaaaaa
kgiocaaaaaaaaaaaalaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaa
aaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
pgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaa
baaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
adaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaa
aaaaaaaaaeaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaa
agaabaaaaaaaaaaaegacbaaaabaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
aaaaaaaaafaaaaaakgakbaaaaaaaaaaaegacbaaaabaaaaaadcaaaaakhccabaaa
aeaaaaaaegiccaaaaaaaaaaaagaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
doaaaaab"
}

SubProgram "gles " {
Keywords { "POINT_COOKIE" }
"!!GLES


#ifdef VERTEX

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _LightTextureB0;
uniform samplerCube _LightTexture0;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_4;
  tmpvar_4 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_7;
  mediump vec3 tmpvar_8;
  tmpvar_8 = normalize(xlv_TEXCOORD2);
  highp float tmpvar_9;
  tmpvar_9 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_LightTextureB0, vec2(tmpvar_9));
  lowp vec4 tmpvar_11;
  tmpvar_11 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_12;
  lightDir_12 = lightDir_2;
  mediump float atten_13;
  atten_13 = (tmpvar_10.w * tmpvar_11.w);
  mediump vec4 c_14;
  highp float nh_15;
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (tmpvar_4, lightDir_12));
  mediump float tmpvar_17;
  tmpvar_17 = max (0.0, dot (tmpvar_4, normalize((lightDir_12 + tmpvar_8))));
  nh_15 = tmpvar_17;
  mediump float tmpvar_18;
  tmpvar_18 = ((1.0 - dot (normalize(tmpvar_8), tmpvar_4)) + dot (normalize(lightDir_12), tmpvar_4));
  highp vec3 tmpvar_19;
  tmpvar_19 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_16) + ((((_LightColor0.xyz * pow (nh_15, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * (atten_13 * 2.0));
  c_14.xyz = tmpvar_19;
  highp vec3 tmpvar_20;
  tmpvar_20 = (c_14.xyz + (c_14.xyz * (pow (tmpvar_18, _RimPower) * _RimColor.xyz)));
  c_14.xyz = tmpvar_20;
  c_14.w = tmpvar_6;
  c_1.xyz = c_14.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "POINT_COOKIE" }
"!!GLES


#ifdef VERTEX

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _LightTextureB0;
uniform samplerCube _LightTexture0;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 normal_4;
  normal_4.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_4.z = sqrt((1.0 - clamp (dot (normal_4.xy, normal_4.xy), 0.0, 1.0)));
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_7;
  mediump vec3 tmpvar_8;
  tmpvar_8 = normalize(xlv_TEXCOORD2);
  highp float tmpvar_9;
  tmpvar_9 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_LightTextureB0, vec2(tmpvar_9));
  lowp vec4 tmpvar_11;
  tmpvar_11 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_12;
  lightDir_12 = lightDir_2;
  mediump float atten_13;
  atten_13 = (tmpvar_10.w * tmpvar_11.w);
  mediump vec4 c_14;
  highp float nh_15;
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (normal_4, lightDir_12));
  mediump float tmpvar_17;
  tmpvar_17 = max (0.0, dot (normal_4, normalize((lightDir_12 + tmpvar_8))));
  nh_15 = tmpvar_17;
  mediump float tmpvar_18;
  tmpvar_18 = ((1.0 - dot (normalize(tmpvar_8), normal_4)) + dot (normalize(lightDir_12), normal_4));
  highp vec3 tmpvar_19;
  tmpvar_19 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_16) + ((((_LightColor0.xyz * pow (nh_15, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * (atten_13 * 2.0));
  c_14.xyz = tmpvar_19;
  highp vec3 tmpvar_20;
  tmpvar_20 = (c_14.xyz + (c_14.xyz * (pow (tmpvar_18, _RimPower) * _RimColor.xyz)));
  c_14.xyz = tmpvar_20;
  c_14.w = tmpvar_6;
  c_1.xyz = c_14.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
Vector 20 [_BumpMap_ST]
"agal_vs
c21 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacbfaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c21.x
aaaaaaaaaaaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c16
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaacaaahacabaaaakeacaaaaaabcaaaappabaaaaaa mul r2.xyz, r1.xyzz, c18.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c17, r0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c17, r1
adaaaaaaabaaahacaeaaaakeacaaaaaabcaaaappabaaaaaa mul r1.xyz, r4.xyzz, c18.w
acaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r1.xyzz, a0
bcaaaaaaabaaacaeaaaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r0.xyzz, r2.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v1.z, a1, r0.xyzz
bcaaaaaaabaaabaeaaaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r0.xyzz, a5
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
bdaaaaaaadaaaeaeaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 v3.z, r0, c14
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaaaaamacadaaaaeeaaaaaaaabeaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c20.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaabeaaaaoeabaaaaaa add v0.zw, r0.wwzw, c20
adaaaaaaaaaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r0.xy, a3, c19
abaaaaaaaaaaadaeaaaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r0.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedoooijienlngbiijahoolihhilkbgbnilabaaaaaahiakaaaaaeaaaaaa
daaaaaaaiaadaaaabaajaaaaniajaaaaebgpgodjeiadaaaaeiadaaaaaaacpopp
niacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaakaaacaaafaaaaaaaaaaabaaaeaaabaaahaaaaaaaaaa
acaaaaaaabaaaiaaaaaaaaaaadaaaaaaaeaaajaaaaaaaaaaadaaamaaajaaanaa
aaaaaaaaaaaaaaaaaaacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaaeaaaaaeaaaaamoaadaaeejaagaaeeka
agaaoekaabaaaaacaaaaapiaaiaaoekaafaaaaadabaaahiaaaaaffiabcaaoeka
aeaaaaaeabaaahiabbaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaakkiaabaaoeiaaeaaaaaeaaaaahiabeaaoekaaaaappiaaaaaoeiaaeaaaaae
aaaaahiaaaaaoeiabfaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaaaaaoeia
abaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaae
abaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappja
aiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeia
abaaaaacaaaaahiaahaaoekaafaaaaadacaaahiaaaaaffiabcaaoekaaeaaaaae
aaaaaliabbaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiabdaaoekaaaaakkia
aaaapeiaacaaaaadaaaaahiaaaaaoeiabeaaoekaaeaaaaaeaaaaahiaaaaaoeia
bfaappkaaaaaoejbaiaaaaadacaaaboaabaaoejaaaaaoeiaaiaaaaadacaaacoa
abaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaapia
aaaaffjaaoaaoekaaeaaaaaeaaaaapiaanaaoekaaaaaaajaaaaaoeiaaeaaaaae
aaaaapiaapaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiabaaaoekaaaaappja
aaaaoeiaafaaaaadabaaahiaaaaaffiaacaaoekaaeaaaaaeabaaahiaabaaoeka
aaaaaaiaabaaoeiaaeaaaaaeaaaaahiaadaaoekaaaaakkiaabaaoeiaaeaaaaae
adaaahoaaeaaoekaaaaappiaaaaaoeiaafaaaaadaaaaapiaaaaaffjaakaaoeka
aeaaaaaeaaaaapiaajaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaalaaoeka
aaaakkjaaaaaoeiaaeaaaaaeaaaaapiaamaaoekaaaaappjaaaaaoeiaaeaaaaae
aaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaa
fdeieefciiafaaaaeaaaabaagcabaaaafjaaaaaeegiocaaaaaaaaaaaamaaaaaa
fjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaa
fjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaad
pcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaa
ghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaad
hccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaa
giaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaa
ogikcaaaaaaaaaaaakaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaa
agiecaaaaaaaaaaaalaaaaaakgiocaaaaaaaaaaaalaaaaaadiaaaaahhcaabaaa
aaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaaaaaaaaaaeaaaaaadcaaaaakhcaabaaaabaaaaaa
egiccaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaakhccabaaaaeaaaaaaegiccaaaaaaaaaaaagaaaaaapgapbaaa
aaaaaaaaegacbaaaaaaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "gles3 " {
Keywords { "POINT_COOKIE" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 403
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 418
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
    highp vec3 _LightCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform highp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform samplerCube _LightTexture0;
uniform highp mat4 _LightMatrix0;
#line 384
uniform sampler2D _LightTextureB0;
uniform highp float _Shininess;
uniform sampler2D _MainTex;
uniform sampler2D _BumpMap;
#line 388
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 410
#line 427
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return ((objSpaceLightPos.xyz * unity_Scale.w) - v.xyz);
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 429
v2f_surf vert_surf( in appdata_full v ) {
    #line 431
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 435
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    o.lightDir = lightDir;
    #line 439
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    o._LightCoord = (_LightMatrix0 * (_Object2World * v.vertex)).xyz;
    #line 443
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out mediump vec3 xlv_TEXCOORD1;
out mediump vec3 xlv_TEXCOORD2;
out highp vec3 xlv_TEXCOORD3;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD2 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD3 = vec3(xl_retval._LightCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 403
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 418
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
    highp vec3 _LightCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform highp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform samplerCube _LightTexture0;
uniform highp mat4 _LightMatrix0;
#line 384
uniform sampler2D _LightTextureB0;
uniform highp float _Shininess;
uniform sampler2D _MainTex;
uniform sampler2D _BumpMap;
#line 388
uniform highp vec4 _RimColor;
uniform highp float _RimPower;
#line 410
#line 427
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 390
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    #line 392
    mediump vec3 h = normalize((lightDir + viewDir));
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    #line 396
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    #line 400
    c.w = s.Alpha;
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 410
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 414
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 445
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 447
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    SurfaceOutput o;
    #line 451
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    o.Alpha = 0.0;
    #line 455
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp vec3 lightDir = normalize(IN.lightDir);
    lowp vec4 c = LightingSimpleSpecular( o, lightDir, normalize(IN.viewDir), ((texture( _LightTextureB0, vec2( dot( IN._LightCoord, IN._LightCoord))).w * texture( _LightTexture0, IN._LightCoord).w) * 1.0));
    #line 459
    c.w = 0.0;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in mediump vec3 xlv_TEXCOORD1;
in mediump vec3 xlv_TEXCOORD2;
in highp vec3 xlv_TEXCOORD3;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD1);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD2);
    xlt_IN._LightCoord = vec3(xlv_TEXCOORD3);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_WorldSpaceLightPos0]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 19 [unity_Scale]
Matrix 13 [_LightMatrix0]
Vector 20 [_MainTex_ST]
Vector 21 [_BumpMap_ST]
"!!ARBvp1.0
# 32 ALU
PARAM c[22] = { { 1 },
		state.matrix.mvp,
		program.local[5..21] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[18];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.y, R0, c[10];
DP4 R3.x, R0, c[9];
DP4 R0.w, vertex.position, c[8];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
DP3 result.texcoord[1].y, R3, R1;
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[1].z, vertex.normal, R3;
DP3 result.texcoord[1].x, R3, vertex.attrib[14];
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
DP4 result.texcoord[3].y, R0, c[14];
DP4 result.texcoord[3].x, R0, c[13];
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[21].xyxy, c[21];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 32 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
Vector 20 [_BumpMap_ST]
"vs_2_0
; 35 ALU
def c21, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c21.x
mov r0.xyz, c16
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c18.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c17, r0
mov r0, c9
dp4 r4.y, c17, r0
mov r1, c8
dp4 r4.x, c17, r1
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 oT1.y, r4, r2
dp3 oT2.y, r2, r3
dp3 oT1.z, v2, r4
dp3 oT1.x, r4, v1
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.zw, v3.xyxy, c20.xyxy, c20
mad oT0.xy, v3, c19, c19.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 20 [_BumpMap_ST]
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 41.33 (31 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~41 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacdiaaaaabomaaaaaaaaaaaaaaceaaaaaaaaaaaaabmmaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaabeaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaap
aaaeaaaaaaaaaapmaaaaaaaaaaaaabamaaacaabdaaabaaaaaaaaaanmaaaaaaaa
aaaaabbiaaacaaagaaaeaaaaaaaaaapmaaaaaaaaaaaaabcgaaacaaakaaaeaaaa
aaaaaapmaaaaaaaaaaaaabdeaaacaaaaaaabaaaaaaaaabemaaaaaaaaaaaaabfm
aaacaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabhbaaacaaacaaaeaaaaaaaaaapm
aaaaaaaaaaaaabieaaacaaaoaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgiheengbhehcgjhi
daaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabomaadbaaahaaaaaaaaaaaaaaaa
aaaadaieaaaaaaabaaaaaaaeaaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaah
aacafaaiaaaapafaaaachbfbaaafhcfcaaaiddfdaaaaaaccaaaabacdaaaaaabm
aaaaaabnaaaababoaaaaaabpaaaaaacaaaaabacbaaaabachpaffeaafaaaabcaa
mcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabj
gabpbcaabcaaaaaaaaaadacfaaaaccaaaaaaaaaaafpihaaaaaaaagiiaaaaaaaa
afpifaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmi
aaaaaaaamiapaaaaaabliiaakbahafaamiapaaaaaamgiiaaklahaeaamiapaaaa
aalbdejeklahadaamiapiadoaagmaadeklahacaamiahaaaaaaleblaacbanabaa
miahaaadaamamgmaalamaaanmiahaaadaalelbleclalaaadmiahaaaeaalogfaa
obacafaamiahaaagaamamgleclamabaamiapaaaaaabliiaakbahajaamiapaaaa
aamgiiaaklahaiaamiahaaagaalelbleclalabagmiahaaaeabgflomaolacafae
miahaaadaamagmleclakaaadmiahaaadabmablmakladaoahmiahaaaeaamablaa
obaeafaamiahaaagaamagmleclakabagmiapaaaaaalbdejeklahahaamiapaaaa
aagmojkkklahagaamiabiaabaaloloaapaagafaamiaciaabaaloloaapaaeagaa
miaeiaabaaloloaapaagacaamiabiaacaaloloaapaadafaamiaciaacaaloloaa
paaeadaamiaeiaacaaloloaapaadacaamiadiaaaaalalabkilabbdbdmiamiaaa
aakmkmagilabbebemiadaaabaalblaaakbaabcaamiadaaabaabllalaklaabbab
miadaaaaaagmlalaklaabaabmiadiaadaamglalaklaaapaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL_COOKIE" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Matrix 268 [_LightMatrix0]
Vector 464 [_MainTex_ST]
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 31 instructions using 5 registers
[Configuration]
8
0000001f41050500
[Microcode]
496
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00019c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01d0700d8106c0c360403ffc
00001c6c01d0600d8106c0c360405ffc00001c6c01d0500d8106c0c360409ffc
00001c6c01d0400d8106c0c360411ffc00021c6c01d0a00d8286c0c360405ffc
00021c6c01d0900d8286c0c360409ffc00021c6c01d0800d8286c0c360411ffc
00009c6c0190a00c0686c0c360405ffc00009c6c0190900c0686c0c360409ffc
00009c6c0190800c0686c0c360411ffc00019c6c00800243011842436041dffc
00011c6c010002308121826301a1dffc401f9c6c01d0d00d8086c0c360409fa8
401f9c6c01d0c00d8086c0c360411fa800001c6c011d100c02bfc0e30041dffc
401f9c6c0140020c0106044360405fa0401f9c6c01400e0c0886008360411fa0
00009c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106004360405fa4
401f9c6c01400e0c0106004360411fa4401f9c6c0140000c0886014360409fa0
401f9c6c0140000c0286004360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 33 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedcgdenkaijkmggdkhopkegkmojjjnjeppabaaaaaapiagaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadamaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcfmafaaaaeaaaabaa
fhabaaaafjaaaaaeegiocaaaaaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaaddccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaalaaaaaa
kgiocaaaaaaaaaaaalaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaa
aaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
pgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaa
beaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
anaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaai
dcaabaaaabaaaaaafgafbaaaaaaaaaaaegiacaaaaaaaaaaaaeaaaaaadcaaaaak
dcaabaaaaaaaaaaaegiacaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegaabaaa
abaaaaaadcaaaaakdcaabaaaaaaaaaaaegiacaaaaaaaaaaaafaaaaaakgakbaaa
aaaaaaaaegaabaaaaaaaaaaadcaaaaakdccabaaaaeaaaaaaegiacaaaaaaaaaaa
agaaaaaapgapbaaaaaaaaaaaegaabaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES


#ifdef VERTEX

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xy;
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 tmpvar_4;
  tmpvar_4 = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).xyz * 2.0) - 1.0);
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  lightDir_2 = xlv_TEXCOORD1;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD2);
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_9;
  lightDir_9 = lightDir_2;
  mediump float atten_10;
  atten_10 = tmpvar_8.w;
  mediump vec4 c_11;
  highp float nh_12;
  mediump float tmpvar_13;
  tmpvar_13 = max (0.0, dot (tmpvar_4, lightDir_9));
  mediump float tmpvar_14;
  tmpvar_14 = max (0.0, dot (tmpvar_4, normalize((lightDir_9 + tmpvar_7))));
  nh_12 = tmpvar_14;
  mediump float tmpvar_15;
  tmpvar_15 = ((1.0 - dot (normalize(tmpvar_7), tmpvar_4)) + dot (normalize(lightDir_9), tmpvar_4));
  highp vec3 tmpvar_16;
  tmpvar_16 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_13) + ((((_LightColor0.xyz * pow (nh_12, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * (atten_10 * 2.0));
  c_11.xyz = tmpvar_16;
  highp vec3 tmpvar_17;
  tmpvar_17 = (c_11.xyz + (c_11.xyz * (pow (tmpvar_15, _RimPower) * _RimColor.xyz)));
  c_11.xyz = tmpvar_17;
  c_11.w = tmpvar_6;
  c_1.xyz = c_11.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES


#ifdef VERTEX

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;
uniform highp mat4 glstate_matrix_mvp;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_6;
  highp vec3 tmpvar_7;
  tmpvar_6 = tmpvar_1.xyz;
  tmpvar_7 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_8;
  tmpvar_8[0].x = tmpvar_6.x;
  tmpvar_8[0].y = tmpvar_7.x;
  tmpvar_8[0].z = tmpvar_2.x;
  tmpvar_8[1].x = tmpvar_6.y;
  tmpvar_8[1].y = tmpvar_7.y;
  tmpvar_8[1].z = tmpvar_2.y;
  tmpvar_8[2].x = tmpvar_6.z;
  tmpvar_8[2].y = tmpvar_7.z;
  tmpvar_8[2].z = tmpvar_2.z;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_9;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8 * (((_World2Object * tmpvar_10).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_5 = tmpvar_11;
  gl_Position = (glstate_matrix_mvp * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xy;
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _RimPower;
uniform highp vec4 _RimColor;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform highp float _Shininess;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _SpecColor;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  lowp vec4 tmpvar_3;
  tmpvar_3 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp vec3 normal_4;
  normal_4.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0.zw).wy * 2.0) - 1.0);
  normal_4.z = sqrt((1.0 - clamp (dot (normal_4.xy, normal_4.xy), 0.0, 1.0)));
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0.xy);
  lowp float tmpvar_6;
  tmpvar_6 = tmpvar_5.w;
  lightDir_2 = xlv_TEXCOORD1;
  mediump vec3 tmpvar_7;
  tmpvar_7 = normalize(xlv_TEXCOORD2);
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_9;
  lightDir_9 = lightDir_2;
  mediump float atten_10;
  atten_10 = tmpvar_8.w;
  mediump vec4 c_11;
  highp float nh_12;
  mediump float tmpvar_13;
  tmpvar_13 = max (0.0, dot (normal_4, lightDir_9));
  mediump float tmpvar_14;
  tmpvar_14 = max (0.0, dot (normal_4, normalize((lightDir_9 + tmpvar_7))));
  nh_12 = tmpvar_14;
  mediump float tmpvar_15;
  tmpvar_15 = ((1.0 - dot (normalize(tmpvar_7), normal_4)) + dot (normalize(lightDir_9), normal_4));
  highp vec3 tmpvar_16;
  tmpvar_16 = ((((tmpvar_3.xyz * _LightColor0.xyz) * tmpvar_13) + ((((_LightColor0.xyz * pow (nh_12, 48.0)) * tmpvar_5.w) * _Shininess) * _SpecColor.xyz)) * (atten_10 * 2.0));
  c_11.xyz = tmpvar_16;
  highp vec3 tmpvar_17;
  tmpvar_17 = (c_11.xyz + (c_11.xyz * (pow (tmpvar_15, _RimPower) * _RimColor.xyz)));
  c_11.xyz = tmpvar_17;
  c_11.w = tmpvar_6;
  c_1.xyz = c_11.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
Vector 20 [_BumpMap_ST]
"agal_vs
c21 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacbfaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c21.x
aaaaaaaaaaaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c16
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaacaaahacabaaaakeacaaaaaabcaaaappabaaaaaa mul r2.xyz, r1.xyzz, c18.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c17, r0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c17, r1
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bcaaaaaaabaaacaeaeaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r4.xyzz, r2.xyzz
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v1.z, a1, r4.xyzz
bcaaaaaaabaaabaeaeaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r4.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaaaaamacadaaaaeeaaaaaaaabeaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c20.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaabeaaaaoeabaaaaaa add v0.zw, r0.wwzw, c20
adaaaaaaaaaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r0.xy, a3, c19
abaaaaaaaaaaadaeaaaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r0.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.zw, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 9 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 33 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_1
eefiecedodkjhpiampnoldjfpdimbdofngogmaglabaaaaaadiakaaaaaeaaaaaa
daaaaaaagmadaaaanaaiaaaajiajaaaaebgpgodjdeadaaaadeadaaaaaaacpopp
meacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaakaaacaaafaaaaaaaaaaabaaaeaaabaaahaaaaaaaaaa
acaaaaaaabaaaiaaaaaaaaaaadaaaaaaaeaaajaaaaaaaaaaadaaamaaajaaanaa
aaaaaaaaaaaaaaaaaaacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaaeaaaaaeaaaaamoaadaaeejaagaaeeka
agaaoekaabaaaaacaaaaapiaaiaaoekaafaaaaadabaaahiaaaaaffiabcaaoeka
aeaaaaaeabaaahiabbaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaakkiaabaaoeiaaeaaaaaeaaaaahiabeaaoekaaaaappiaaaaaoeiaaiaaaaad
abaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaabaaoejaafaaaaadacaaahia
abaamjiaacaancjaaeaaaaaeabaaahiaacaamjjaabaanciaacaaoeibafaaaaad
abaaahiaabaaoeiaabaappjaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaad
abaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaahaaoekaafaaaaadacaaahia
aaaaffiabcaaoekaaeaaaaaeaaaaaliabbaakekaaaaaaaiaacaakeiaaeaaaaae
aaaaahiabdaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeiabeaaoeka
aeaaaaaeaaaaahiaaaaaoeiabfaappkaaaaaoejbaiaaaaadacaaaboaabaaoeja
aaaaoeiaaiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoeja
aaaaoeiaafaaaaadaaaaapiaaaaaffjaaoaaoekaaeaaaaaeaaaaapiaanaaoeka
aaaaaajaaaaaoeiaaeaaaaaeaaaaapiaapaaoekaaaaakkjaaaaaoeiaaeaaaaae
aaaaapiabaaaoekaaaaappjaaaaaoeiaafaaaaadabaaadiaaaaaffiaacaaoeka
aeaaaaaeaaaaadiaabaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaadiaadaaoeka
aaaakkiaaaaaoeiaaeaaaaaeadaaadoaaeaaoekaaaaappiaaaaaoeiaafaaaaad
aaaaapiaaaaaffjaakaaoekaaeaaaaaeaaaaapiaajaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaalaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaamaaoeka
aaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaac
aaaaammaaaaaoeiappppaaaafdeieefcfmafaaaaeaaaabaafhabaaaafjaaaaae
egiocaaaaaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaae
egiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
pccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaaddccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaalaaaaaakgiocaaaaaaaaaaa
alaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaa
abaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaidcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiacaaaaaaaaaaaaeaaaaaadcaaaaakdcaabaaaaaaaaaaa
egiacaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegaabaaaabaaaaaadcaaaaak
dcaabaaaaaaaaaaaegiacaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegaabaaa
aaaaaaaadcaaaaakdccabaaaaeaaaaaaegiacaaaaaaaaaaaagaaaaaapgapbaaa
aaaaaaaaegaabaaaaaaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadamaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES3#version 300 es


#ifdef VERTEX

#define gl_Vertex _glesVertex
in vec4 _glesVertex;
#define gl_Color _glesColor
in vec4 _glesColor;
#define gl_Normal (normalize(_glesNormal))
in vec3 _glesNormal;
#define gl_MultiTexCoord0 _glesMultiTexCoord0
in vec4 _glesMultiTexCoord0;
#define gl_MultiTexCoord1 _glesMultiTexCoord1
in vec4 _glesMultiTexCoord1;
#define TANGENT vec4(normalize(_glesTANGENT.xyz), _glesTANGENT.w)
in vec4 _glesTANGENT;
mat2 xll_transpose_mf2x2(mat2 m) {
  return mat2( m[0][0], m[1][0], m[0][1], m[1][1]);
}
mat3 xll_transpose_mf3x3(mat3 m) {
  return mat3( m[0][0], m[1][0], m[2][0],
               m[0][1], m[1][1], m[2][1],
               m[0][2], m[1][2], m[2][2]);
}
mat4 xll_transpose_mf4x4(mat4 m) {
  return mat4( m[0][0], m[1][0], m[2][0], m[3][0],
               m[0][1], m[1][1], m[2][1], m[3][1],
               m[0][2], m[1][2], m[2][2], m[3][2],
               m[0][3], m[1][3], m[2][3], m[3][3]);
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 402
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 417
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
    highp vec2 _LightCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform sampler2D _LightTexture0;
uniform highp mat4 _LightMatrix0;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
#line 388
uniform highp float _RimPower;
#line 409
#line 426
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 81
highp vec3 ObjSpaceLightDir( in highp vec4 v ) {
    highp vec3 objSpaceLightPos = (_World2Object * _WorldSpaceLightPos0).xyz;
    return objSpaceLightPos.xyz;
}
#line 90
highp vec3 ObjSpaceViewDir( in highp vec4 v ) {
    highp vec3 objSpaceCameraPos = ((_World2Object * vec4( _WorldSpaceCameraPos.xyz, 1.0)).xyz * unity_Scale.w);
    return (objSpaceCameraPos - v.xyz);
}
#line 428
v2f_surf vert_surf( in appdata_full v ) {
    #line 430
    v2f_surf o;
    o.pos = (glstate_matrix_mvp * v.vertex);
    o.pack0.xy = ((v.texcoord.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
    o.pack0.zw = ((v.texcoord.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
    #line 434
    highp vec3 binormal = (cross( v.normal, v.tangent.xyz) * v.tangent.w);
    highp mat3 rotation = xll_transpose_mf3x3(mat3( v.tangent.xyz, binormal, v.normal));
    highp vec3 lightDir = (rotation * ObjSpaceLightDir( v.vertex));
    o.lightDir = lightDir;
    #line 438
    highp vec3 viewDirForLight = (rotation * ObjSpaceViewDir( v.vertex));
    o.viewDir = viewDirForLight;
    o._LightCoord = (_LightMatrix0 * (_Object2World * v.vertex)).xy;
    #line 442
    return o;
}

out highp vec4 xlv_TEXCOORD0;
out mediump vec3 xlv_TEXCOORD1;
out mediump vec3 xlv_TEXCOORD2;
out highp vec2 xlv_TEXCOORD3;
void main() {
    v2f_surf xl_retval;
    appdata_full xlt_v;
    xlt_v.vertex = vec4(gl_Vertex);
    xlt_v.tangent = vec4(TANGENT);
    xlt_v.normal = vec3(gl_Normal);
    xlt_v.texcoord = vec4(gl_MultiTexCoord0);
    xlt_v.texcoord1 = vec4(gl_MultiTexCoord1);
    xlt_v.color = vec4(gl_Color);
    xl_retval = vert_surf( xlt_v);
    gl_Position = vec4(xl_retval.pos);
    xlv_TEXCOORD0 = vec4(xl_retval.pack0);
    xlv_TEXCOORD1 = vec3(xl_retval.lightDir);
    xlv_TEXCOORD2 = vec3(xl_retval.viewDir);
    xlv_TEXCOORD3 = vec2(xl_retval._LightCoord);
}


#endif
#ifdef FRAGMENT

#define gl_FragData _glesFragData
layout(location = 0) out mediump vec4 _glesFragData[4];
float xll_saturate_f( float x) {
  return clamp( x, 0.0, 1.0);
}
vec2 xll_saturate_vf2( vec2 x) {
  return clamp( x, 0.0, 1.0);
}
vec3 xll_saturate_vf3( vec3 x) {
  return clamp( x, 0.0, 1.0);
}
vec4 xll_saturate_vf4( vec4 x) {
  return clamp( x, 0.0, 1.0);
}
mat2 xll_saturate_mf2x2(mat2 m) {
  return mat2( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0));
}
mat3 xll_saturate_mf3x3(mat3 m) {
  return mat3( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0));
}
mat4 xll_saturate_mf4x4(mat4 m) {
  return mat4( clamp(m[0], 0.0, 1.0), clamp(m[1], 0.0, 1.0), clamp(m[2], 0.0, 1.0), clamp(m[3], 0.0, 1.0));
}
#line 150
struct v2f_vertex_lit {
    highp vec2 uv;
    lowp vec4 diff;
    lowp vec4 spec;
};
#line 186
struct v2f_img {
    highp vec4 pos;
    mediump vec2 uv;
};
#line 180
struct appdata_img {
    highp vec4 vertex;
    mediump vec2 texcoord;
};
#line 306
struct SurfaceOutput {
    lowp vec3 Albedo;
    lowp vec3 Normal;
    lowp vec3 Emission;
    mediump float Specular;
    lowp float Gloss;
    lowp float Alpha;
};
#line 402
struct Input {
    highp vec2 uv_MainTex;
    highp vec2 uv_BumpMap;
    highp vec3 viewDir;
};
#line 417
struct v2f_surf {
    highp vec4 pos;
    highp vec4 pack0;
    mediump vec3 lightDir;
    mediump vec3 viewDir;
    highp vec2 _LightCoord;
};
#line 66
struct appdata_full {
    highp vec4 vertex;
    highp vec4 tangent;
    highp vec3 normal;
    highp vec4 texcoord;
    highp vec4 texcoord1;
    lowp vec4 color;
};
uniform highp vec4 _Time;
uniform highp vec4 _SinTime;
#line 3
uniform highp vec4 _CosTime;
uniform highp vec4 unity_DeltaTime;
uniform highp vec3 _WorldSpaceCameraPos;
uniform highp vec4 _ProjectionParams;
#line 7
uniform highp vec4 _ScreenParams;
uniform highp vec4 _ZBufferParams;
uniform highp vec4 unity_CameraWorldClipPlanes[6];
uniform lowp vec4 _WorldSpaceLightPos0;
#line 11
uniform highp vec4 _LightPositionRange;
uniform highp vec4 unity_4LightPosX0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosZ0;
#line 15
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_LightPosition[4];
uniform highp vec4 unity_LightAtten[4];
#line 19
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHBr;
#line 23
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHC;
uniform highp vec3 unity_LightColor0;
uniform highp vec3 unity_LightColor1;
uniform highp vec3 unity_LightColor2;
uniform highp vec3 unity_LightColor3;
#line 27
uniform highp vec4 unity_ShadowSplitSpheres[4];
uniform highp vec4 unity_ShadowSplitSqRadii;
uniform highp vec4 unity_LightShadowBias;
uniform highp vec4 _LightSplitsNear;
#line 31
uniform highp vec4 _LightSplitsFar;
uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 _LightShadowData;
uniform highp vec4 unity_ShadowFadeCenterAndType;
#line 35
uniform highp mat4 glstate_matrix_mvp;
uniform highp mat4 glstate_matrix_modelview0;
uniform highp mat4 glstate_matrix_invtrans_modelview0;
uniform highp mat4 _Object2World;
#line 39
uniform highp mat4 _World2Object;
uniform highp vec4 unity_Scale;
uniform highp mat4 glstate_matrix_transpose_modelview0;
uniform highp mat4 glstate_matrix_texture0;
#line 43
uniform highp mat4 glstate_matrix_texture1;
uniform highp mat4 glstate_matrix_texture2;
uniform highp mat4 glstate_matrix_texture3;
uniform highp mat4 glstate_matrix_projection;
#line 47
uniform highp vec4 glstate_lightmodel_ambient;
uniform highp mat4 unity_MatrixV;
uniform highp mat4 unity_MatrixVP;
uniform lowp vec4 unity_ColorSpaceGrey;
#line 76
#line 81
#line 86
#line 90
#line 95
#line 119
#line 136
#line 157
#line 165
#line 192
#line 205
#line 214
#line 219
#line 228
#line 233
#line 242
#line 259
#line 264
#line 290
#line 298
#line 302
#line 316
uniform lowp vec4 _LightColor0;
uniform lowp vec4 _SpecColor;
#line 329
#line 337
#line 351
uniform sampler2D _LightTexture0;
uniform highp mat4 _LightMatrix0;
#line 384
uniform highp float _Shininess;
uniform sampler2D _MainTex;
uniform sampler2D _BumpMap;
uniform highp vec4 _RimColor;
#line 388
uniform highp float _RimPower;
#line 409
#line 426
uniform highp vec4 _MainTex_ST;
uniform highp vec4 _BumpMap_ST;
#line 389
mediump vec4 LightingSimpleSpecular( in SurfaceOutput s, in mediump vec3 lightDir, in mediump vec3 viewDir, in mediump float atten ) {
    mediump vec3 h = normalize((lightDir + viewDir));
    #line 392
    mediump float diff = max( 0.0, dot( s.Normal, lightDir));
    highp float nh = max( 0.0, dot( s.Normal, h));
    highp float spec = pow( nh, 48.0);
    mediump float rim = ((1.0 - dot( normalize(viewDir), s.Normal)) + dot( normalize(lightDir), s.Normal));
    #line 396
    mediump vec4 c;
    c.xyz = ((((s.Albedo * _LightColor0.xyz) * diff) + ((((_LightColor0.xyz * spec) * s.Alpha) * _Shininess) * vec3( _SpecColor))) * (atten * 2.0));
    c.xyz = (c.xyz + (c.xyz * (pow( rim, _RimPower) * _RimColor.xyz)));
    c.w = s.Alpha;
    #line 400
    return c;
}
#line 271
lowp vec3 UnpackNormal( in lowp vec4 packednormal ) {
    #line 273
    return ((packednormal.xyz * 2.0) - 1.0);
}
#line 409
void surf( in Input IN, inout SurfaceOutput o ) {
    o.Albedo = texture( _MainTex, IN.uv_MainTex).xyz;
    o.Normal = UnpackNormal( texture( _BumpMap, IN.uv_BumpMap));
    #line 413
    mediump float rim = (1.0 - xll_saturate_f(dot( normalize(IN.viewDir), o.Normal)));
    o.Emission = (_RimColor.xyz * pow( rim, _RimPower));
    o.Alpha = texture( _MainTex, IN.uv_MainTex).w;
}
#line 444
lowp vec4 frag_surf( in v2f_surf IN ) {
    #line 446
    Input surfIN;
    surfIN.uv_MainTex = IN.pack0.xy;
    surfIN.uv_BumpMap = IN.pack0.zw;
    SurfaceOutput o;
    #line 450
    o.Albedo = vec3( 0.0);
    o.Emission = vec3( 0.0);
    o.Specular = 0.0;
    o.Alpha = 0.0;
    #line 454
    o.Gloss = 0.0;
    surf( surfIN, o);
    lowp vec3 lightDir = IN.lightDir;
    lowp vec4 c = LightingSimpleSpecular( o, lightDir, normalize(IN.viewDir), (texture( _LightTexture0, IN._LightCoord).w * 1.0));
    #line 458
    c.w = 0.0;
    return c;
}
in highp vec4 xlv_TEXCOORD0;
in mediump vec3 xlv_TEXCOORD1;
in mediump vec3 xlv_TEXCOORD2;
in highp vec2 xlv_TEXCOORD3;
void main() {
    lowp vec4 xl_retval;
    v2f_surf xlt_IN;
    xlt_IN.pos = vec4(0.0);
    xlt_IN.pack0 = vec4(xlv_TEXCOORD0);
    xlt_IN.lightDir = vec3(xlv_TEXCOORD1);
    xlt_IN.viewDir = vec3(xlv_TEXCOORD2);
    xlt_IN._LightCoord = vec2(xlv_TEXCOORD3);
    xl_retval = frag_surf( xlt_IN);
    gl_FragData[0] = vec4(xl_retval);
}


#endif"
}

}
Program "fp" {
// Fragment combos: 5
//   opengl - ALU: 41 to 53, TEX: 2 to 4
//   d3d9 - ALU: 50 to 62, TEX: 2 to 4
//   d3d11 - ALU: 34 to 40, TEX: 2 to 4, FLOW: 1 to 1
//   d3d11_9x - ALU: 34 to 40, TEX: 2 to 4, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 47 ALU, 3 TEX
PARAM c[6] = { program.local[0..4],
		{ 0, 2, 1, 48 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R1.yw, fragment.texcoord[0].zwzw, texture[1], 2D;
DP3 R1.x, fragment.texcoord[3], fragment.texcoord[3];
DP3 R1.z, fragment.texcoord[2], fragment.texcoord[2];
RSQ R1.z, R1.z;
MUL R3.xyz, R1.z, fragment.texcoord[2];
DP3 R3.w, R3, R3;
RSQ R3.w, R3.w;
MUL R0.xyz, R0, c[0];
MOV result.color.w, c[5].x;
TEX R2.w, R1.x, texture[2], 2D;
DP3 R1.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R1.x, R1.x;
MUL R2.xyz, R1.x, fragment.texcoord[1];
MAD R1.xy, R1.wyzw, c[5].y, -c[5].z;
MUL R1.zw, R1.xyxy, R1.xyxy;
ADD R4.xyz, R2, R3;
ADD_SAT R1.z, R1, R1.w;
DP3 R1.w, R4, R4;
RSQ R1.w, R1.w;
ADD R1.z, -R1, c[5];
RSQ R1.z, R1.z;
MUL R4.xyz, R1.w, R4;
RCP R1.z, R1.z;
DP3 R1.w, R1, R4;
MUL R3.xyz, R3.w, R3;
MAX R1.w, R1, c[5].x;
DP3 R4.x, R2, R2;
RSQ R4.x, R4.x;
MUL R4.xyz, R4.x, R2;
DP3 R3.x, R1, R3;
DP3 R3.w, R1, R4;
POW R1.w, R1.w, c[5].w;
ADD R3.w, -R3.x, R3;
MUL R3.xyz, R1.w, c[0];
DP3 R1.w, R1, R2;
MUL R1.xyz, R3, R0.w;
MAX R0.w, R1, c[5].x;
MUL R0.xyz, R0, R0.w;
MUL R1.xyz, R1, c[2].x;
MAD R0.xyz, R1, c[1], R0;
ADD R1.x, R3.w, c[5].z;
MUL R0.w, R2, c[5].y;
POW R1.x, R1.x, c[4].x;
MUL R0.xyz, R0, R0.w;
MUL R1.xyz, R1.x, c[3];
MAD result.color.xyz, R0, R1, R0;
END
# 47 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
"ps_2_0
; 54 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 48.00000000, 0, 0, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
texld r2, t0, s0
dp3 r0.x, t3, t3
mov r0.xy, r0.x
dp3_pp r3.x, t2, t2
dp3_pp r4.x, t1, t1
rsq_pp r3.x, r3.x
rsq_pp r4.x, r4.x
mul_pp r4.xyz, r4.x, t1
mul_pp r3.xyz, r3.x, t2
add_pp r5.xyz, r4, r3
dp3_pp r6.x, r5, r5
rsq_pp r6.x, r6.x
mul_pp r5.xyz, r6.x, r5
dp3_pp r6.x, r4, r4
mov r1.y, t0.w
mov r1.x, t0.z
rsq_pp r6.x, r6.x
mul_pp r2.xyz, r2, c0
texld r0, r0, s2
texld r1, r1, s1
mov r1.x, r1.w
mad_pp r7.xy, r1, c5.x, c5.y
mul_pp r1.xy, r7, r7
add_pp_sat r1.x, r1, r1.y
add_pp r1.x, -r1, c5.z
rsq_pp r1.x, r1.x
rcp_pp r7.z, r1.x
dp3_pp r1.x, r7, r5
max_pp r1.x, r1, c5.w
pow r5.w, r1.x, c6.x
mov r1.x, r5.w
dp3_pp r5.x, r3, r3
rsq_pp r5.x, r5.x
mul_pp r3.xyz, r5.x, r3
mul_pp r5.xyz, r6.x, r4
mul r1.xyz, r1.x, c0
mul r1.xyz, r1, r2.w
dp3_pp r4.x, r7, r4
max_pp r4.x, r4, c5.w
mul_pp r2.xyz, r2, r4.x
mul r1.xyz, r1, c2.x
mad r1.xyz, r1, c1, r2
mul_pp r0.x, r0, c5
mul r0.xyz, r1, r0.x
dp3_pp r3.x, r7, r3
dp3_pp r5.x, r7, r5
add_pp r3.x, -r3, r5
add_pp r3.x, r3, c5.z
pow_pp r2.x, r3.x, c4.x
mul r1.xyz, r2.x, c3
mad r0.xyz, r0, r1, r0
mov_pp r0.w, c5
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 3 [_RimColor]
Float 4 [_RimPower]
Float 2 [_Shininess]
Vector 1 [_SpecColor]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 40.00 (30 instructions), vertex: 0, texture: 12,
//   sequencer: 14, interpolator: 16;    6 GPRs, 30 threads,
// Performance (if enough threads): ~40 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaaboaaaaaacaiaaaaaaaaaaaaaaceaaaaabiiaaaaablaaaaaaaaa
aaaaaaaaaaaaabgaaaaaaabmaaaaabfbppppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabekaaaaaalmaaadaaacaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaaa
aaabaaaaaaaaaaoiaaaaaaaaaaaaaapiaaadaaaaaaabaaaaaaaaaamiaaaaaaaa
aaaaabahaaadaaabaaabaaaaaaaaaamiaaaaaaaaaaaaabbaaaacaaadaaabaaaa
aaaaaaoiaaaaaaaaaaaaabbkaaacaaaeaaabaaaaaaaaabceaaaaaaaaaaaaabde
aaacaaacaaabaaaaaaaaabceaaaaaaaaaaaaabdpaaacaaabaaabaaaaaaaaaaoi
aaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpemgjghgiheedgpgmgphcdaaaklklklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpemgjghgihefegfhihehfhcgfdaaafpengbgjgofegfhiaafpfcgjgnedgpgmgp
hcaafpfcgjgnfagphhgfhcaaaaaaaaadaaabaaabaaabaaaaaaaaaaaafpfdgigj
gogjgogfhdhdaafpfdhagfgdedgpgmgphcaahahdfpddfpdaaadccodacodcdadd
dfddcodaaaklklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaaba
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabmibaaaafaaaaaaaaae
aaaaaaaaaaaadeieaaapaaapaaaaaaabaaaapafaaaaahbfbaaaahcfcaaaahdfd
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
eaaaaaaaaaaaaaaalpiaaaaaeceaaaaadpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aafeeaaeaaaabcaameaaaaaaaaaagaaigaaobcaabcaaaaaaaaaagabegabkbcaa
bcaaaaaaaaaafacaaaaaccaaaaaaaaaamiaiaaabaaloloaapaadadaababidaab
bpbppgiiaaaaeaaapmaieacbbpbppppiaaaaeaaadiciaaabbpbppopjaaaaeaaa
miacaaaaaaloloaapaacacaamiadaaafaagogmmgilaapopoficbaaaaaalololb
paababiafiboaaaeaalbpmgmobaaaciamiahaaacaagmmaaaobaaabaamjacaaaa
aalalalbnbafafpomiaiaaacaaloloaapaacacaamiaoaaabaapmabaaoaacaeaa
liebaaaaaamdmdebnaaeaeppfibbaaabaamdmdgmpaababiafiilacaaaabfgmbl
obaeaaickaeoafaeaapmblmgobacaciamiaeaaaeaamdloaapaaeafaafibeabaa
aalologmpaacafibmiahaaabaabfgmaaobababaamiacaaaaaalploaapaaaafaa
miabaaaaaaloloaapaabafaalickaeabaammlbebicaapoppeabfaaabaagngolb
oaaeaeibeacbaaaaaagmblgmkbaapoibdibcaaaaaalbgmgmkbaaaeaadiihaaac
aamamalbkbadaaaamiahaaaaaagmmaaakbaaaaaamiahaaaaaamablaaobaaadaa
kibhabaaaamagmedibaaacadkichabaaaamamaidibaaabadmiahaaaaaamablma
olacabaakiehabaaaamgmamdmbabaaadmiahmaaaaamamamaolaaabaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
"sce_fp_rsx // 52 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840004000000
[Offsets]
5
_LightColor0 2 0
00000250000000b0
_SpecColor 1 0
000002a0
_Shininess 1 0
00000230
_RimColor 1 0
00000300
_RimPower 1 0
000002d0
[Microcode]
832
ee020100c8011c9dc8000001c8003fe110060500c8041c9dc8040001c8000001
940017025c011c9dc8000001c8003fe106820440ce001c9d00020000aa020000
000040000000bf800000000000000000ae843940c8011c9dc8000029c800bfe1
1080b840c9041c9dc9040001c8000001ce863940c8011c9dc8000029c800bfe1
0e060340c9081c9dc90c0001c80000019e041700c8011c9dc8000001c8003fe1
0e800240c8081c9dc8020001c800000100000000000000000000000000000000
10800340c9001c9fc8020001c800000100000000000000000000000000003f80
08823b40ff003c9dff000001c80000010e8c3940c80c1c9dc8000029c8000001
10800540c9041c9dc9180001c800000102041704fe0c1c9dc8000001c8000001
10060900c9001c9d00020000c800000100000000000000000000000000000000
0e863940c90c1c9dc8000029c800000110840540c9041c9dc90c0001c8000001
0e863940c9081c9dc8000029c800000102840540c9041c9dc9080001c8000001
10800540c9041c9dc90c0001c800000102061d00fe0c1c9dc8000001c8000001
02820340ff081c9fff000001c800000110000200000c1c9c00020000c8000001
0000424000000000000000000000000004021c00fe001c9dc8000001c8000001
04040200c8041c9dfe080001c8000001108a090001081c9c00020000c8000001
000000000000000000000000000000000e8a0240c9001c9dff140001c8000001
10000200aa081c9c00020000c800000100000000000000000000000000000000
0e020200fe001c9dc8020001c800000100000000000000000000000000000000
1080034001041c9cc8020001c800000100000000000000000000000000003f80
10021d00ff001c9dc8000001c80000010e020400c8041c9dc8021001c9140001
000000000000000000000000000000000e000200c8041c9d00080000c8000001
10000200c8041c9d00020000c800000100000000000000000000000000000000
10001c00fe001c9dc8000001c800000116020200fe001c9d48020001c8000001
000000000000000000000000000000000e800400c8001c9d68040001c8000001
1081014000021c9cc8000001c800000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "POINT" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_BumpMap] 2D 2
SetTexture 2 [_LightTexture0] 2D 0
// 44 instructions, 5 temp regs, 0 temp arrays:
// ALU 35 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedlgipdjigifpagdblbmbnaeinkfnmgppkabaaaaaajiagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcjaafaaaaeaaaaaaageabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaa
ffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaa
gcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaa
aeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegbcbaaa
adaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaa
eeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadcaaaaajhcaabaaaabaaaaaa
egbcbaaaacaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadiaaaaahhcaabaaa
acaaaaaapgapbaaaaaaaaaaaegbcbaaaacaaaaaabaaaaaahicaabaaaaaaaaaaa
egacbaaaabaaaaaaegacbaaaabaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaa
aaaaaaaadiaaaaahhcaabaaaabaaaaaapgapbaaaaaaaaaaaegacbaaaabaaaaaa
efaaaaajpcaabaaaadaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
acaaaaaadcaaaaapdcaabaaaadaaaaaahgapbaaaadaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
apaaaaahicaabaaaaaaaaaaaegaabaaaadaaaaaaegaabaaaadaaaaaaddaaaaah
icaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaa
aaaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaa
adaaaaaadkaabaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaadaaaaaa
egacbaaaabaaaaaadeaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaa
aaaaaaaacpaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaeaecbjaaaaaficaabaaaaaaaaaaa
dkaabaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaapgapbaaaaaaaaaaaegiccaaa
aaaaaaaaabaaaaaaefaaaaajpcaabaaaaeaaaaaaegbabaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgapbaaaaeaaaaaadiaaaaaihcaabaaaaeaaaaaaegacbaaaaeaaaaaaegiccaaa
aaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaagiacaaa
aaaaaaaaahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
aaaaaaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaadaaaaaaegacbaaa
acaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaadaaaaaa
aaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadp
aaaaaaahbcaabaaaaaaaaaaadkaabaaaaaaaaaaaakaabaaaaaaaaaaadeaaaaah
ccaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaajocaabaaa
aaaaaaaaagajbaaaaeaaaaaafgafbaaaaaaaaaaaagajbaaaabaaaaaacpaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaibcaabaaaaaaaaaaaakaabaaa
aaaaaaaaakiacaaaaaaaaaaaajaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaaihcaabaaaabaaaaaaagaabaaaaaaaaaaaegiccaaaaaaaaaaa
aiaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaa
efaaaaajpcaabaaaacaaaaaaagaabaaaaaaaaaaaeghobaaaacaaaaaaaagabaaa
aaaaaaaaaaaaaaahbcaabaaaaaaaaaaaakaabaaaacaaaaaaakaabaaaacaaaaaa
diaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaadcaaaaaj
hccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "POINT" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "POINT" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
"agal_ps
c5 2.0 -1.0 1.0 0.0
c6 48.0 0.0 0.0 0.0
[bc]
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
bcaaaaaaaaaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r0.x, v3, v3
aaaaaaaaaaaaadacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.xy, r0.x
bcaaaaaaadaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r3.x, v2, v2
akaaaaaaadaaabacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r3.x, r3.x
adaaaaaaaeaaahacadaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r4.xyz, r3.x, v2
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
aaaaaaaaabaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r1.y, v0.w
aaaaaaaaabaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r1.x, v0.z
ciaaaaaaabaaapacabaaaafeacaaaaaaabaaaaaaafaababb tex r1, r1.xyyy, s1 <2d wrap linear point>
ciaaaaaaaaaaapacaaaaaafeacaaaaaaacaaaaaaafaababb tex r0, r0.xyyy, s2 <2d wrap linear point>
bcaaaaaaabaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, v1, v1
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaadaaahacabaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r1.x, v1
abaaaaaaagaaahacadaaaakeacaaaaaaaeaaaakeacaaaaaa add r6.xyz, r3.xyzz, r4.xyzz
bcaaaaaaabaaabacagaaaakeacaaaaaaagaaaakeacaaaaaa dp3 r1.x, r6.xyzz, r6.xyzz
aaaaaaaaaaaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r1.y
aaaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.w
adaaaaaaafaaadacaaaaaafeacaaaaaaafaaaaaaabaaaaaa mul r5.xy, r0.xyyy, c5.x
abaaaaaaafaaadacafaaaafeacaaaaaaafaaaaffabaaaaaa add r5.xy, r5.xyyy, c5.y
adaaaaaaaaaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r0.x, r5.y, r5.y
bfaaaaaaadaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r3.w, r5.x
adaaaaaaadaaaiacadaaaappacaaaaaaafaaaaaaacaaaaaa mul r3.w, r3.w, r5.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r3.w, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
adaaaaaaabaaahacabaaaaaaacaaaaaaagaaaakeacaaaaaa mul r1.xyz, r1.x, r6.xyzz
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r1.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
alaaaaaaabaaapacaaaaaaaaacaaaaaaagaaaaaaabaaaaaa pow r1, r0.x, c6.x
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
bcaaaaaaabaaabacaeaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r4.xyzz
adaaaaaaaaaaahacaaaaaaaaacaaaaaaaaaaaaoeabaaaaaa mul r0.xyz, r0.x, c0
adaaaaaaaaaaahacaaaaaakeacaaaaaaacaaaappacaaaaaa mul r0.xyz, r0.xyzz, r2.w
adaaaaaaagaaahacaaaaaakeacaaaaaaacaaaaaaabaaaaaa mul r6.xyz, r0.xyzz, c2.x
bcaaaaaaaaaaabacadaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r0.x, r3.xyzz, r3.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaaeaaaakeacaaaaaa mul r1.xyz, r1.x, r4.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaaeaaahacaaaaaaaaacaaaaaaadaaaakeacaaaaaa mul r4.xyz, r0.x, r3.xyzz
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r1.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r4.xyzz
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaabaaaaaaacaaaaaa add r0.x, r0.x, r1.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
bcaaaaaaadaaabacafaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r3.x, r5.xyzz, r3.xyzz
ahaaaaaaabaaabacadaaaaaaacaaaaaaafaaaappabaaaaaa max r1.x, r3.x, c5.w
adaaaaaaabaaahacacaaaakeacaaaaaaabaaaaaaacaaaaaa mul r1.xyz, r2.xyzz, r1.x
adaaaaaaacaaahacagaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r6.xyzz, c1
abaaaaaaacaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa add r2.xyz, r2.xyzz, r1.xyzz
alaaaaaaabaaapacaaaaaaaaacaaaaaaaeaaaaaaabaaaaaa pow r1, r0.x, c4.x
adaaaaaaadaaabacaaaaaappacaaaaaaafaaaaoeabaaaaaa mul r3.x, r0.w, c5
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
adaaaaaaabaaahacacaaaakeacaaaaaaadaaaaaaacaaaaaa mul r1.xyz, r2.xyzz, r3.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaadaaaaoeabaaaaaa mul r0.xyz, r0.x, c3
adaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r1.xyzz, r0.xyzz
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaaiacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c5
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_BumpMap] 2D 2
SetTexture 2 [_LightTexture0] 2D 0
// 44 instructions, 5 temp regs, 0 temp arrays:
// ALU 35 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_1
eefiecedanlkimpbgckokkbdcgpaopacklggcfpdabaaaaaaoiajaaaaaeaaaaaa
daaaaaaahmadaaaabeajaaaaleajaaaaebgpgodjeeadaaaaeeadaaaaaaacpppp
pmacaaaaeiaaaaaaacaadaaaaaaaeiaaaaaaeiaaadaaceaaaaaaeiaaacaaaaaa
aaababaaabacacaaaaaaabaaacaaaaaaaaaaaaaaaaaaahaaadaaacaaaaaaaaaa
aaacppppfbaaaaafafaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaiadpfbaaaaaf
agaaapkaaaaaeaecaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaapla
bpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaacaaaaaaia
adaaahlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaac
aaaaaajaacaiapkaabaaaaacaaaaabiaaaaakklaabaaaaacaaaaaciaaaaappla
aiaaaaadabaaaiiaadaaoelaadaaoelaabaaaaacabaaadiaabaappiaecaaaaad
aaaacpiaaaaaoeiaacaioekaecaaaaadacaaapiaaaaaoelaabaioekaecaaaaad
abaacpiaabaaoeiaaaaioekaaeaaaaaeadaacbiaaaaappiaafaaaakaafaaffka
aeaaaaaeadaacciaaaaaffiaafaaaakaafaaffkafkaaaaaeadaadiiaadaaoeia
adaaoeiaafaakkkaacaaaaadadaaciiaadaappibafaappkaahaaaaacadaaciia
adaappiaagaaaaacadaaceiaadaappiaaiaaaaadadaaciiaabaaoelaabaaoela
ahaaaaacadaaciiaadaappiaceaaaaacaaaachiaacaaoelaaeaaaaaeaeaachia
abaaoelaadaappiaaaaaoeiaafaaaaadafaachiaadaappiaabaaoelaaiaaaaad
aaaaciiaadaaoeiaafaaoeiaaiaaaaadadaaciiaaaaaoeiaadaaoeiaacaaaaad
adaaciiaaaaappiaadaappibalaaaaadaeaaciiaaaaappiaafaakkkaacaaaaad
adaaciiaadaappiaafaappkacaaaaaadaaaaabiaadaappiaaeaaaakaafaaaaad
aaaaahiaaaaaaaiaadaaoekaceaaaaacafaachiaaeaaoeiaaiaaaaadaaaaciia
adaaoeiaafaaoeiaalaaaaadabaaaciaaaaappiaafaakkkacaaaaaadaaaaaiia
abaaffiaagaaaakaafaaaaadabaaaoiaaaaappiaaaaablkaafaaaaadabaaaoia
acaappiaabaaoeiaafaaaaadacaachiaacaaoeiaaaaaoekaafaaaaadabaaaoia
abaaoeiaacaaaakaafaaaaadabaaaoiaabaaoeiaabaablkaaeaaaaaeabaaaoia
acaabliaaeaappiaabaaoeiaacaaaaadaaaaaiiaabaaaaiaabaaaaiaafaaaaad
abaachiaaaaappiaabaabliaaeaaaaaeaaaachiaabaaoeiaaaaaoeiaabaaoeia
abaaaaacaaaaciiaafaakkkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefc
jaafaaaaeaaaaaaageabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaafkaaaaad
aagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegbcbaaaadaaaaaa
baaaaaahicaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaf
icaabaaaaaaaaaaadkaabaaaaaaaaaaadcaaaaajhcaabaaaabaaaaaaegbcbaaa
acaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadiaaaaahhcaabaaaacaaaaaa
pgapbaaaaaaaaaaaegbcbaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaabaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaa
diaaaaahhcaabaaaabaaaaaapgapbaaaaaaaaaaaegacbaaaabaaaaaaefaaaaaj
pcaabaaaadaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaa
dcaaaaapdcaabaaaadaaaaaahgapbaaaadaaaaaaaceaaaaaaaaaaaeaaaaaaaea
aaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaaapaaaaah
icaabaaaaaaaaaaaegaabaaaadaaaaaaegaabaaaadaaaaaaddaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaaaaaaaaaa
dkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaaadaaaaaa
dkaabaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaadaaaaaaegacbaaa
abaaaaaadeaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaa
cpaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahicaabaaaaaaaaaaa
dkaabaaaaaaaaaaaabeaaaaaaaaaeaecbjaaaaaficaabaaaaaaaaaaadkaabaaa
aaaaaaaadiaaaaaihcaabaaaabaaaaaapgapbaaaaaaaaaaaegiccaaaaaaaaaaa
abaaaaaaefaaaaajpcaabaaaaeaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaa
aagabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaa
aeaaaaaadiaaaaaihcaabaaaaeaaaaaaegacbaaaaeaaaaaaegiccaaaaaaaaaaa
abaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaagiacaaaaaaaaaaa
ahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaa
acaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaadaaaaaaegacbaaaacaaaaaa
baaaaaahbcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaadaaaaaaaaaaaaai
bcaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaah
bcaabaaaaaaaaaaadkaabaaaaaaaaaaaakaabaaaaaaaaaaadeaaaaahccaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaa
agajbaaaaeaaaaaafgafbaaaaaaaaaaaagajbaaaabaaaaaacpaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaaibcaabaaaaaaaaaaaakaabaaaaaaaaaaa
akiacaaaaaaaaaaaajaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaaihcaabaaaabaaaaaaagaabaaaaaaaaaaaegiccaaaaaaaaaaaaiaaaaaa
baaaaaahbcaabaaaaaaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaaefaaaaaj
pcaabaaaacaaaaaaagaabaaaaaaaaaaaeghobaaaacaaaaaaaagabaaaaaaaaaaa
aaaaaaahbcaabaaaaaaaaaaaakaabaaaacaaaaaaakaabaaaacaaaaaadiaaaaah
hcaabaaaaaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaadcaaaaajhccabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaadgaaaaaf
iccabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahahaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklkl"
}

SubProgram "gles3 " {
Keywords { "POINT" }
"!!GLES3"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 41 ALU, 2 TEX
PARAM c[6] = { program.local[0..4],
		{ 0, 2, 1, 48 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R1.yw, fragment.texcoord[0].zwzw, texture[1], 2D;
DP3 R1.x, fragment.texcoord[2], fragment.texcoord[2];
RSQ R1.z, R1.x;
MUL R2.xyz, R1.z, fragment.texcoord[2];
MAD R1.xy, R1.wyzw, c[5].y, -c[5].z;
MUL R1.zw, R1.xyxy, R1.xyxy;
ADD_SAT R1.z, R1, R1.w;
ADD R3.xyz, fragment.texcoord[1], R2;
DP3 R1.w, R3, R3;
RSQ R1.w, R1.w;
DP3 R2.w, R2, R2;
RSQ R2.w, R2.w;
ADD R1.z, -R1, c[5];
RSQ R1.z, R1.z;
MUL R2.xyz, R2.w, R2;
RCP R1.z, R1.z;
MUL R3.xyz, R1.w, R3;
DP3 R1.w, R1, R3;
DP3 R3.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R3.x, R3.x;
MUL R3.xyz, R3.x, fragment.texcoord[1];
DP3 R2.x, R1, R2;
DP3 R2.w, R1, R3;
ADD R2.y, -R2.x, R2.w;
MAX R1.w, R1, c[5].x;
POW R2.x, R1.w, c[5].w;
ADD R1.w, R2.y, c[5].z;
DP3 R2.w, R1, fragment.texcoord[1];
MUL R2.xyz, R2.x, c[0];
MUL R1.xyz, R2, R0.w;
MAX R0.w, R2, c[5].x;
MUL R0.xyz, R0, c[0];
MUL R0.xyz, R0, R0.w;
MUL R1.xyz, R1, c[2].x;
MAD R0.xyz, R1, c[1], R0;
POW R0.w, R1.w, c[4].x;
MUL R0.xyz, R0, c[5].y;
MUL R1.xyz, R0.w, c[3];
MAD result.color.xyz, R0, R1, R0;
MOV result.color.w, c[5].x;
END
# 41 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
"ps_2_0
; 50 ALU, 2 TEX
dcl_2d s0
dcl_2d s1
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 48.00000000, 0, 0, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
texld r3, t0, s0
mov r0.y, t0.w
mov r0.x, t0.z
mul_pp r3.xyz, r3, c0
texld r0, r0, s1
mov r1.x, r0.w
mov r1.y, r0
dp3_pp r0.x, t2, t2
rsq_pp r0.x, r0.x
mul_pp r4.xyz, r0.x, t2
mad_pp r5.xy, r1, c5.x, c5.y
add_pp r2.xyz, t1, r4
dp3_pp r1.x, r2, r2
mul_pp r0.xy, r5, r5
add_pp_sat r0.x, r0, r0.y
add_pp r0.x, -r0, c5.z
rsq_pp r0.x, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r2
rcp_pp r5.z, r0.x
dp3_pp r0.x, r5, r1
dp3_pp r1.x, r4, r4
rsq_pp r1.x, r1.x
dp3_pp r2.x, t1, t1
rsq_pp r2.x, r2.x
mul_pp r2.xyz, r2.x, t1
mul_pp r4.xyz, r1.x, r4
dp3_pp r1.x, r5, r2
dp3_pp r2.x, r5, r4
add_pp r1.x, -r2, r1
max_pp r0.x, r0, c5.w
pow r2.w, r0.x, c6.x
add_pp r0.x, r1, c5.z
pow_pp r1.w, r0.x, c4.x
mov r0.x, r2.w
mul r2.xyz, r0.x, c0
dp3_pp r0.x, r5, t1
mul r2.xyz, r2, r3.w
max_pp r0.x, r0, c5.w
mul_pp r0.xyz, r3, r0.x
mul r2.xyz, r2, c2.x
mad r2.xyz, r2, c1, r0
mov_pp r0.x, r1.w
mul r1.xyz, r2, c5.x
mul r0.xyz, r0.x, c3
mad r0.xyz, r1, r0, r1
mov_pp r0.w, c5
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 3 [_RimColor]
Float 4 [_RimPower]
Float 2 [_Shininess]
Vector 1 [_SpecColor]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 34.67 (26 instructions), vertex: 0, texture: 8,
//   sequencer: 14, interpolator: 12;    7 GPRs, 27 threads,
// Performance (if enough threads): ~34 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaablmaaaaabmmaaaaaaaaaaaaaaceaaaaabgiaaaaabjaaaaaaaaa
aaaaaaaaaaaaabeaaaaaaabmaaaaabdbppppadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabckaaaaaakiaaadaaabaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaaa
aaabaaaaaaaaaaneaaaaaaaaaaaaaaoeaaadaaaaaaabaaaaaaaaaaleaaaaaaaa
aaaaaaonaaacaaadaaabaaaaaaaaaaneaaaaaaaaaaaaaaphaaacaaaeaaabaaaa
aaaaabaeaaaaaaaaaaaaabbeaaacaaacaaabaaaaaaaaabaeaaaaaaaaaaaaabbp
aaacaaabaaabaaaaaaaaaaneaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaam
aaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcdaaaklklklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhiaafpfcgjgnedgpgmgphcaafp
fcgjgnfagphhgfhcaaklklklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpfdgigj
gogjgogfhdhdaafpfdhagfgdedgpgmgphcaahahdfpddfpdaaadccodacodcdadd
dfddcodaaaklklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaaba
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabimbaaaagaaaaaaaaae
aaaaaaaaaaaacigdaaahaaahaaaaaaabaaaapafaaaaahbfbaaaahcfcaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaa
aaaaaaaalpiaaaaaeceaaaaadpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaafcaae
aaaabcaameaaaaaaaaaagaaggaambcaabcaaaaaaaaaagabcgabibcaabcaaaaaa
aaaacaboaaaaccaaaaaaaaaabaaidaabbpbppgiiaaaaeaaadibiaaabbpbppopj
aaaaeaaamiacaaaaaaloloaapaababaamiadaaaeaagogmmgilaapopoficbaaaa
aalololbpaacaciafibhaaafaalbmagmobaaabiamiahaaagaagmmaaaobaaacaa
mjacaaaaaalalalbnbaeaepobeihaeacaamamagmmaagabppliebaaaaaaloloeb
naagagppfibiaaabaalologmpaacaciakaelaeaaaamagmmgobagaaiafibeabaa
aaloloblpaaeabibmiahaaabaamagmaaobacabaamiacaaaaaalploaapaaaaeaa
miabaaaaaaloloaapaabaeaaliifafabaamelbebicaapoppeabcaaaaaadodogm
opafaeibeacbaaaaaagmbllbkbaapoiadibcaaaaaalbgmgmkbaaaeaadiihaaac
aalelelbkbadaaaamiahaaaaaagmmaaakbaaaaaamiahaaaaaamablaaobaaadaa
kibhabaaaalegmedibaaacadkichabaaaamalemdibaaabadmiahaaaaaamamgma
olacabaakiehabaaaaleleidmaaaaaadmiahmaaaaalemamaolabaaaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
"sce_fp_rsx // 48 instructions using 4 registers
[Configuration]
24
ffffffff0001c0200007fff8000000000000840004000000
[Offsets]
5
_LightColor0 2 0
00000230000000c0
_SpecColor 1 0
00000290
_Shininess 1 0
00000210
_RimColor 1 0
000002c0
_RimPower 1 0
00000260
[Microcode]
768
940017025c011c9dc8000001c8003fe106860440ce001c9d00020000aa020000
000040000000bf8000000000000000009e061700c8011c9dc8000001c8003fe1
ce803940c8011c9dc8000029c800bfe11080b840c90c1c9dc90c0001c8000001
10800340c9001c9f00020000c800000100003f80000000000000000000000000
ae820140c8011c9dc8000001c8003fe10e840340c9041c9dc9000001c8000001
08863b40ff003c9dff000001c80000010e8c0240c80c1c9dc8020001c8000001
000000000000000000000000000000000e843940c9081c9dc8000029c8000001
10800540c90c1c9dc9080001c800000110040900c9001c9d00020000c8000001
0000000000000000000000000000000002880540c90c1c9dc9040001c8000001
08001d00fe081c9dc8000001c8000001ae843940c8011c9dc8000029c8003fe1
108c090001101c9c00020000c800000100000000000000000000000000000000
08060200c8001c9daa020000c800000100000000000042400000000000000000
0e803940c9001c9dc8000029c800000102800540c90c1c9dc9000001c8000001
08061c00540c1c9dc8000001c800000110000200540c1c9dc80c0001c8000001
04800540c90c1c9dc9080001c800000102800340c9001c9fab000000c8000001
1084034001001c9caa020000c80000010000000000003f800000000000000000
02020200fe001c9d00020000c800000100000000000000000000000000000000
0e00020000041c9cc8020001c800000100000000000000000000000000000000
04021d00ff081c9dc8000001c800000110000200aa041c9c00020000c8000001
000000000000000000000000000000000e860240c9181c9dff180001c8000001
0e000400c8001c9dc8021001c90c000100000000000000000000000000000000
10001c00fe001c9dc8000001c800000116020200fe001c9d48020001c8000001
000000000000000000000000000000000e800400c8001c9d68040001c8000001
1081014000021c9cc8000001c800000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" }
ConstBuffer "$Globals" 128 // 84 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 48 [_Shininess]
Vector 64 [_RimColor] 4
Float 80 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_BumpMap] 2D 1
// 42 instructions, 4 temp regs, 0 temp arrays:
// ALU 34 float, 0 int, 0 uint
// TEX 2 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedecegkekblgpdbpeblieddgjccfcijjloabaaaaaabiagaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcciafaaaaeaaaaaaaekabaaaafjaaaaaeegiocaaa
aaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
gcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaa
adaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaaagbjbaaaadaaaaaaagaabaaa
aaaaaaaaagbjbaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaa
egbcbaaaadaaaaaabaaaaaahbcaabaaaaaaaaaaajgahbaaaaaaaaaaajgahbaaa
aaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaa
ogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaadcaaaaapdcaabaaa
acaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaaapaaaaahicaabaaaaaaaaaaa
egaabaaaacaaaaaaegaabaaaacaaaaaaddaaaaahicaabaaaaaaaaaaadkaabaaa
aaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaaaaaaaaaadkaabaiaebaaaaaa
aaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaaacaaaaaadkaabaaaaaaaaaaa
baaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaaaaaaaaadeaaaaah
bcaabaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
abeaaaaaaaaaeaecbjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaai
hcaabaaaaaaaaaaaagaabaaaaaaaaaaaegiccaaaaaaaaaaaabaaaaaaefaaaaaj
pcaabaaaadaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaadaaaaaadiaaaaai
hcaabaaaadaaaaaaegacbaaaadaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaai
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaagiacaaaaaaaaaaaadaaaaaadiaaaaai
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaacaaaaaabaaaaaah
icaabaaaaaaaaaaaegacbaaaacaaaaaaegbcbaaaacaaaaaadeaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaadaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaaaaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaa
egacbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaaiicaabaaaaaaaaaaadkaabaia
ebaaaaaaaaaaaaaaabeaaaaaaaaaiadpbaaaaaahbcaabaaaabaaaaaaegbcbaaa
acaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaa
diaaaaahhcaabaaaabaaaaaaagaabaaaabaaaaaaegbcbaaaacaaaaaabaaaaaah
bcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaakaabaaaabaaaaaacpaaaaaficaabaaaaaaaaaaa
dkaabaaaaaaaaaaadiaaaaaiicaabaaaaaaaaaaadkaabaaaaaaaaaaaakiacaaa
aaaaaaaaafaaaaaabjaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaai
hcaabaaaabaaaaaapgapbaaaaaaaaaaaegiccaaaaaaaaaaaaeaaaaaadcaaaaaj
hccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
"agal_ps
c5 2.0 -1.0 1.0 0.0
c6 48.0 0.0 0.0 0.0
[bc]
ciaaaaaaadaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r3, v0, s0 <2d wrap linear point>
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
adaaaaaaadaaahacadaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r3.xyz, r3.xyzz, c0
ciaaaaaaaaaaapacaaaaaafeacaaaaaaabaaaaaaafaababb tex r0, r0.xyyy, s1 <2d wrap linear point>
aaaaaaaaabaaabacaaaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r0.w
aaaaaaaaabaaacacaaaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r1.y, r0.y
bcaaaaaaaaaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, v2, v2
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaaeaaahacaaaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r4.xyz, r0.x, v2
adaaaaaaafaaadacabaaaafeacaaaaaaafaaaaaaabaaaaaa mul r5.xy, r1.xyyy, c5.x
abaaaaaaafaaadacafaaaafeacaaaaaaafaaaaffabaaaaaa add r5.xy, r5.xyyy, c5.y
abaaaaaaacaaahacabaaaaoeaeaaaaaaaeaaaakeacaaaaaa add r2.xyz, v1, r4.xyzz
bcaaaaaaabaaabacacaaaakeacaaaaaaacaaaakeacaaaaaa dp3 r1.x, r2.xyzz, r2.xyzz
adaaaaaaaaaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r0.x, r5.y, r5.y
bfaaaaaaabaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.w, r5.x
adaaaaaaabaaaiacabaaaappacaaaaaaafaaaaaaacaaaaaa mul r1.w, r1.w, r5.x
acaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.w, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaacaaaakeacaaaaaa mul r1.xyz, r1.x, r2.xyzz
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r1.xyzz
bcaaaaaaabaaabacaeaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r4.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
bcaaaaaaacaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r2.x, v1, v1
akaaaaaaacaaabacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r2.x, r2.x
adaaaaaaacaaahacacaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r2.xyz, r2.x, v1
adaaaaaaaeaaahacabaaaaaaacaaaaaaaeaaaakeacaaaaaa mul r4.xyz, r1.x, r4.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaacaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r2.xyzz
bcaaaaaaacaaabacafaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r2.x, r5.xyzz, r4.xyzz
bfaaaaaaacaaaiacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r2.w, r2.x
abaaaaaaabaaabacacaaaappacaaaaaaabaaaaaaacaaaaaa add r1.x, r2.w, r1.x
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
alaaaaaaacaaapacaaaaaaaaacaaaaaaagaaaaaaabaaaaaa pow r2, r0.x, c6.x
abaaaaaaaaaaabacabaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r1.x, c5.z
alaaaaaaabaaapacaaaaaaaaacaaaaaaaeaaaaaaabaaaaaa pow r1, r0.x, c4.x
aaaaaaaaaaaaabacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r2.x
adaaaaaaacaaahacaaaaaaaaacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r0.x, c0
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, r5.xyzz, v1
adaaaaaaacaaahacacaaaakeacaaaaaaadaaaappacaaaaaa mul r2.xyz, r2.xyzz, r3.w
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
adaaaaaaaaaaahacadaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r3.xyzz, r0.x
adaaaaaaacaaahacacaaaakeacaaaaaaacaaaaaaabaaaaaa mul r2.xyz, r2.xyzz, c2.x
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
abaaaaaaacaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa add r2.xyz, r2.xyzz, r0.xyzz
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
adaaaaaaabaaahacacaaaakeacaaaaaaafaaaaaaabaaaaaa mul r1.xyz, r2.xyzz, c5.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaadaaaaoeabaaaaaa mul r0.xyz, r0.x, c3
adaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r1.xyzz, r0.xyzz
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaaiacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c5
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" }
ConstBuffer "$Globals" 128 // 84 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 48 [_Shininess]
Vector 64 [_RimColor] 4
Float 80 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_BumpMap] 2D 1
// 42 instructions, 4 temp regs, 0 temp arrays:
// ALU 34 float, 0 int, 0 uint
// TEX 2 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_1
eefiecedgcljlkdfhnaanlakelbmllbjmijmlbimabaaaaaacaajaaaaaeaaaaaa
daaaaaaadeadaaaageaiaaaaomaiaaaaebgpgodjpmacaaaapmacaaaaaaacpppp
meacaaaadiaaaaaaabaacmaaaaaadiaaaaaadiaaacaaceaaaaaadiaaaaaaaaaa
abababaaaaaaabaaafaaaaaaaaaaaaaaaaacppppfbaaaaafafaaapkaaaaaaaea
aaaaialpaaaaaaaaaaaaiadpfbaaaaafagaaapkaaaaaeaecaaaaaaaaaaaaaaaa
aaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaac
aaaaaaiaacaachlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapka
aiaaaaadaaaaciiaacaaoelaacaaoelaahaaaaacaaaacbiaaaaappiaabaaaaac
abaaahiaacaaoelaaeaaaaaeabaachiaabaaoeiaaaaaaaiaabaaoelaafaaaaad
aaaachiaaaaaaaiaacaaoelaceaaaaacacaachiaabaaoeiaabaaaaacabaaabia
aaaakklaabaaaaacabaaaciaaaaapplaecaaaaadabaacpiaabaaoeiaabaioeka
ecaaaaadadaaapiaaaaaoelaaaaioekaaeaaaaaeaeaacbiaabaappiaafaaaaka
afaaffkaaeaaaaaeaeaacciaabaaffiaafaaaakaafaaffkafkaaaaaeaaaadiia
aeaaoeiaaeaaoeiaafaakkkaacaaaaadaaaaciiaaaaappibafaappkaahaaaaac
aaaaciiaaaaappiaagaaaaacaeaaceiaaaaappiaaiaaaaadaaaaciiaaeaaoeia
acaaoeiaalaaaaadaeaaaiiaaaaappiaafaakkkacaaaaaadaaaaaiiaaeaappia
agaaaakaafaaaaadabaaahiaaaaappiaaaaaoekaafaaaaadabaaahiaadaappia
abaaoeiaafaaaaadacaachiaadaaoeiaaaaaoekaafaaaaadabaaahiaabaaoeia
acaaaakaafaaaaadabaaahiaabaaoeiaabaaoekaaiaaaaadaaaaciiaaeaaoeia
abaaoelaalaaaaadabaaciiaaaaappiaafaakkkaaeaaaaaeabaaahiaacaaoeia
abaappiaabaaoeiaacaaaaadabaachiaabaaoeiaabaaoeiaaiaaaaadabaaciia
aaaaoeiaaeaaoeiaacaaaaadabaaciiaabaappibafaappkaceaaaaacaaaachia
abaaoelaaiaaaaadaaaacbiaaaaaoeiaaeaaoeiaacaaaaadabaaciiaaaaaaaia
abaappiacaaaaaadaaaaabiaabaappiaaeaaaakaafaaaaadaaaaahiaaaaaaaia
adaaoekaaeaaaaaeaaaachiaabaaoeiaaaaaoeiaabaaoeiaabaaaaacaaaaciia
afaakkkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcciafaaaaeaaaaaaa
ekabaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaa
fkaaaaadaagabaaaabaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaac
aeaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaa
eeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaa
agbjbaaaadaaaaaaagaabaaaaaaaaaaaagbjbaaaacaaaaaadiaaaaahhcaabaaa
abaaaaaaagaabaaaaaaaaaaaegbcbaaaadaaaaaabaaaaaahbcaabaaaaaaaaaaa
jgahbaaaaaaaaaaajgahbaaaaaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaa
efaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
abaaaaaadcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
apaaaaahicaabaaaaaaaaaaaegaabaaaacaaaaaaegaabaaaacaaaaaaddaaaaah
icaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaa
aaaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaa
acaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaa
egacbaaaaaaaaaaadeaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaeaecbjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadiaaaaaihcaabaaaaaaaaaaaagaabaaaaaaaaaaaegiccaaa
aaaaaaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaegbabaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaa
pgapbaaaadaaaaaadiaaaaaihcaabaaaadaaaaaaegacbaaaadaaaaaaegiccaaa
aaaaaaaaabaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaagiacaaa
aaaaaaaaadaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaa
aaaaaaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaacaaaaaaegbcbaaa
acaaaaaadeaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaa
dcaaaaajhcaabaaaaaaaaaaaegacbaaaadaaaaaapgapbaaaaaaaaaaaegacbaaa
aaaaaaaaaaaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaa
baaaaaahicaabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaai
icaabaaaaaaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpbaaaaaah
bcaabaaaabaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaa
abaaaaaaakaabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaabaaaaaa
egbcbaaaacaaaaaabaaaaaahbcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaakaabaaaabaaaaaa
cpaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaaiicaabaaaaaaaaaaa
dkaabaaaaaaaaaaaakiacaaaaaaaaaaaafaaaaaabjaaaaaficaabaaaaaaaaaaa
dkaabaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaapgapbaaaaaaaaaaaegiccaaa
aaaaaaaaaeaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaa
doaaaaabejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaaaaaaaaaaabaaaaaa
adaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaa
apapaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaheaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL" }
"!!GLES3"
}

SubProgram "opengl " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
SetTexture 3 [_LightTextureB0] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 53 ALU, 4 TEX
PARAM c[7] = { program.local[0..4],
		{ 0, 2, 1, 48 },
		{ 0.5 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R2, fragment.texcoord[0], texture[0], 2D;
TEX R3.yw, fragment.texcoord[0].zwzw, texture[1], 2D;
MAD R4.xy, R3.wyzw, c[5].y, -c[5].z;
DP3 R0.z, fragment.texcoord[3], fragment.texcoord[3];
MUL R4.zw, R4.xyxy, R4.xyxy;
ADD_SAT R3.w, R4.z, R4;
RCP R0.x, fragment.texcoord[3].w;
MAD R0.xy, fragment.texcoord[3], R0.x, c[6].x;
DP3 R1.x, fragment.texcoord[2], fragment.texcoord[2];
RSQ R1.x, R1.x;
ADD R3.w, -R3, c[5].z;
MUL R3.xyz, R1.x, fragment.texcoord[2];
RSQ R3.w, R3.w;
MUL R2.xyz, R2, c[0];
MOV result.color.w, c[5].x;
TEX R0.w, R0, texture[2], 2D;
TEX R1.w, R0.z, texture[3], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.x;
MUL R0.xyz, R0.x, fragment.texcoord[1];
ADD R1.xyz, R0, R3;
DP3 R4.z, R1, R1;
RSQ R4.z, R4.z;
MUL R1.xyz, R4.z, R1;
RCP R4.z, R3.w;
DP3 R1.x, R4, R1;
MAX R1.x, R1, c[5];
POW R1.x, R1.x, c[5].w;
MUL R1.xyz, R1.x, c[0];
MUL R1.xyz, R1, R2.w;
DP3 R3.w, R4, R0;
MAX R2.w, R3, c[5].x;
MUL R2.xyz, R2, R2.w;
MUL R1.xyz, R1, c[2].x;
MAD R1.xyz, R1, c[1], R2;
DP3 R2.y, R0, R0;
RSQ R2.y, R2.y;
MUL R0.xyz, R2.y, R0;
DP3 R2.x, R3, R3;
RSQ R2.x, R2.x;
DP3 R0.y, R4, R0;
MUL R2.xyz, R2.x, R3;
DP3 R0.x, R4, R2;
ADD R0.y, -R0.x, R0;
SLT R0.x, c[5], fragment.texcoord[3].z;
MUL R0.x, R0, R0.w;
ADD R0.y, R0, c[5].z;
MUL R0.x, R0, R1.w;
POW R0.w, R0.y, c[4].x;
MUL R0.x, R0, c[5].y;
MUL R0.xyz, R1, R0.x;
MUL R1.xyz, R0.w, c[3];
MAD result.color.xyz, R0, R1, R0;
END
# 53 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
SetTexture 3 [_LightTextureB0] 2D
"ps_2_0
; 62 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 48.00000000, 0.50000000, 0, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3
texld r3, t0, s0
dp3 r1.x, t3, t3
mov r1.xy, r1.x
dp3_pp r4.x, t1, t1
rsq_pp r4.x, r4.x
mul_pp r4.xyz, r4.x, t1
mov r0.y, t0.w
mov r0.x, t0.z
mov r2.xy, r0
rcp r0.x, t3.w
mad r0.xy, t3, r0.x, c6.y
mul_pp r3.xyz, r3, c0
texld r2, r2, s1
texld r1, r1, s3
texld r0, r0, s2
dp3_pp r2.x, t2, t2
mov r0.y, r2
mov r0.x, r2.w
mad_pp r7.xy, r0, c5.x, c5.y
mul_pp r0.xy, r7, r7
add_pp_sat r0.x, r0, r0.y
rsq_pp r2.x, r2.x
mul_pp r2.xyz, r2.x, t2
add_pp r5.xyz, r4, r2
dp3_pp r6.x, r5, r5
rsq_pp r6.x, r6.x
mul_pp r5.xyz, r6.x, r5
dp3_pp r6.x, r4, r4
add_pp r0.x, -r0, c5.z
rsq_pp r0.x, r0.x
rcp_pp r7.z, r0.x
dp3_pp r0.x, r7, r5
max_pp r0.x, r0, c5.w
pow r5.w, r0.x, c6.x
mov r0.x, r5.w
dp3_pp r5.x, r2, r2
rsq_pp r5.x, r5.x
mul_pp r2.xyz, r5.x, r2
mul r0.xyz, r0.x, c0
mul r0.xyz, r0, r3.w
rsq_pp r6.x, r6.x
mul_pp r5.xyz, r6.x, r4
dp3_pp r4.x, r7, r4
max_pp r4.x, r4, c5.w
mul_pp r3.xyz, r3, r4.x
mul r0.xyz, r0, c2.x
mad r0.xyz, r0, c1, r3
dp3_pp r2.x, r7, r2
dp3_pp r5.x, r7, r5
add_pp r2.x, -r2, r5
add_pp r2.x, r2, c5.z
pow_pp r3.x, r2.x, c4.x
cmp r2.x, -t3.z, c5.w, c5.z
mul_pp r2.x, r2, r0.w
mul_pp r1.x, r2, r1
mul_pp r1.x, r1, c5
mul r0.xyz, r0, r1.x
mov_pp r2.x, r3.x
mul r1.xyz, r2.x, c3
mad r0.xyz, r0, r1, r0
mov_pp r0.w, c5
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 3 [_RimColor]
Float 4 [_RimPower]
Float 2 [_Shininess]
Vector 1 [_SpecColor]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_LightTextureB0] 2D
SetTexture 2 [_MainTex] 2D
SetTexture 3 [_BumpMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 41.33 (31 instructions), vertex: 0, texture: 16,
//   sequencer: 14, interpolator: 16;    8 GPRs, 24 threads,
// Performance (if enough threads): ~41 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacaeaaaaaccaaaaaaaaaaaaaaaceaaaaabkmaaaaabneaaaaaaaa
aaaaaaaaaaaaabieaaaaaabmaaaaabhfppppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabgoaaaaaanaaaadaaadaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaaa
aaabaaaaaaaaaapmaaaaaaaaaaaaabamaaadaaaaaaabaaaaaaaaaanmaaaaaaaa
aaaaabblaaadaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabclaaadaaacaaabaaaa
aaaaaanmaaaaaaaaaaaaabdeaaacaaadaaabaaaaaaaaaapmaaaaaaaaaaaaabdo
aaacaaaeaaabaaaaaaaaabeiaaaaaaaaaaaaabfiaaacaaacaaabaaaaaaaaabei
aaaaaaaaaaaaabgdaaacaaabaaabaaaaaaaaaapmaaaaaaaafpechfgnhaengbha
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcda
aaklklklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgihefegfhihehfhc
gfdaaafpemgjghgihefegfhihehfhcgfecdaaafpengbgjgofegfhiaafpfcgjgn
edgpgmgphcaafpfcgjgnfagphhgfhcaaaaaaaaadaaabaaabaaabaaaaaaaaaaaa
fpfdgigjgogjgogfhdhdaafpfdhagfgdedgpgmgphcaahahdfpddfpdaaadccoda
codcdadddfddcodaaaklklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
abpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaboabaaaahaa
aaaaaaaeaaaaaaaaaaaadiieaaapaaapaaaaaaabaaaapafaaaaahbfbaaaahcfc
aaaapdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaeaaaaaaadpiaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaalpiaaaaa
eceaaaaaaffagaaeaaaabcaameaaaaaaaaaagaakgababcaabcaaaaaaaaaagabg
gabmbcaabcaaaaaaaaaafaccaaaaccaaaaaaaaaaemiiacabaaloloblpaadadad
miadaaaeaabllagmmlacadpppmbibacbbpbppbppaaaaeaaabaaigaibbpbpphpp
aaaaeaaabacieaabbpbppgiiaaaaeaaadidiaaabbpbppghpaaaaeaaamiabaaaa
aaloloaapaacacaamiacaaaaaaloloaapaababaamiadaaafaamhgmmgilaapopp
fichaaaeaamamalbkbaeaaiafichaaahaalbmagmobaaabiamjabaaaaaalalalb
nbafafppmiahaaagaalbmaaaobaaacaamiabaaabaaloloaapaagagaamiahaaac
aamamaaaoaahagaaliccabaaaaloloianaahahpoficbaaaaaalololbpaacacia
kaeoafaaaapmlblbobahaaibcaeiabaaaamdlomgpaaaafadfibcaaabaalologm
paahafiafibhabaaaamagmgmobacaaibmiapaaacaaaaomaaobagabaabeabaaab
aaloloblpaaaafacamcbaaaaaaloloblpaacafabliekaaabaalmlbiaicabpppo
eabfaaabaalcldlboaaaaaibeacbaaaaaagmblgmkbaappibdibiaaaaaalbgmgm
kbaaaeaadiihaaaaaagmmablkbaaaaaamiahaaaaaamablaaobaaaeaakibhabaa
aamagmedibaaacadkichabaaaamamaidibaaabadmiahaaaaaamablmaolaeabaa
kiehabaaaamgmamdmbabaaadmiahmaaaaamamamaolaaabaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
SetTexture 3 [_LightTextureB0] 2D
"sce_fp_rsx // 60 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840004000000
[Offsets]
5
_LightColor0 2 0
000002f000000110
_SpecColor 1 0
00000310
_Shininess 1 0
000002b0
_RimColor 1 0
00000380
_RimPower 1 0
00000340
[Microcode]
960
fe040100c8011c9dc8000001c8003fe110060500c8081c9dc8080001c8000001
940017025c011c9dc8000001c8003fe106820440ce001c9d00020000aa020000
000040000000bf8000000000000000000882b840c9041c9dc9040001c8000001
06003a00c8081c9dfe080001c800000108820340c9041c9f00020000c8000001
00003f8000000000000000000000000006000300c8001c9daa020000c8000001
0000000000003f00000000000000000010041704c8001c9dc8000001c8000001
ae883940c8011c9dc8000029c800bfe1028a0d0054081c9d00020000c8000001
000000000000000000000000000000009e021700c8011c9dc8000001c8003fe1
0e840240c8041c9dc8020001c800000100000000000000000000000000000000
1088024001141c9cc8080001c800000102001706fe0c1c9dc8000001c8000001
ce8a3940c8011c9dc8000029c800bfe102800240ff101c9dc8001001c8000001
08823b40c9043c9d55040001c80000010e060340c9101c9dc9140001c8000001
0e8c3940c80c1c9dc8000029c800000108800540c9041c9dc9180001c8000001
0e8a3940c9141c9dc8000029c800000102860540c9041c9dc9140001c8000001
0400090055001c9d00020000c800000100000000000000000000000000000000
10840540c9041c9dc9100001c80000010e8a3940c9101c9dc8000029c8000001
02041d00aa001c9cc8000001c800000104860540c9041c9dc9140001c8000001
1000020000081c9c00020000c800000100004240000000000000000000000000
04001c00fe001c9dc8000001c800000102860340c90c1c9fab0c0000c8000001
04000200c8001c9dfe040001c800000110820900c9081c9d00020000c8000001
000000000000000000000000000000000e820240c9081c9dff040001c8000001
10040200aa001c9c00020000c800000100000000000000000000000000000000
10860340010c1c9cc8020001c800000100000000000000000000000000003f80
0e020200fe081c9dc8020001c800000100000000000000000000000000000000
0e020400c8041c9dc8020001c904000100000000000000000000000000000000
04001d00ff0c1c9dc8000001c800000110000200aa001c9c00020000c8000001
000000000000000000000000000000000e000200c8041c9d01000000c8000001
10001c00fe001c9dc8000001c800000116020200fe001c9d48020001c8000001
000000000000000000000000000000000e800400c8001c9d68040001c8000001
1081014000021c9cc8000001c800000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "SPOT" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_BumpMap] 2D 3
SetTexture 2 [_LightTexture0] 2D 0
SetTexture 3 [_LightTextureB0] 2D 1
// 50 instructions, 5 temp regs, 0 temp arrays:
// ALU 39 float, 0 int, 1 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedohechdmillmpnecijkmomoifkmmgenbcabaaaaaahaahaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcgiagaaaaeaaaaaaajkabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadpcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaaaoaaaaahdcaabaaaaaaaaaaa
egbabaaaaeaaaaaapgbpbaaaaeaaaaaaaaaaaaakdcaabaaaaaaaaaaaegaabaaa
aaaaaaaaaceaaaaaaaaaaadpaaaaaadpaaaaaaaaaaaaaaaaefaaaaajpcaabaaa
aaaaaaaaegaabaaaaaaaaaaaeghobaaaacaaaaaaaagabaaaaaaaaaaadbaaaaah
bcaabaaaaaaaaaaaabeaaaaaaaaaaaaackbabaaaaeaaaaaaabaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaiadpdiaaaaahbcaabaaaaaaaaaaa
dkaabaaaaaaaaaaaakaabaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaa
aeaaaaaaegbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaafgafbaaaaaaaaaaa
eghobaaaadaaaaaaaagabaaaabaaaaaaapaaaaahbcaabaaaaaaaaaaaagaabaaa
aaaaaaaaagaabaaaabaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaadaaaaaa
egbcbaaaadaaaaaaeeaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaah
ocaabaaaaaaaaaaafgafbaaaaaaaaaaaagbjbaaaadaaaaaabaaaaaahbcaabaaa
abaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaabaaaaaa
akaabaaaabaaaaaadcaaaaajocaabaaaabaaaaaaagbjbaaaacaaaaaaagaabaaa
abaaaaaafgaobaaaaaaaaaaadiaaaaahhcaabaaaacaaaaaaagaabaaaabaaaaaa
egbcbaaaacaaaaaabaaaaaahbcaabaaaabaaaaaajgahbaaaabaaaaaajgahbaaa
abaaaaaaeeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahhcaabaaa
abaaaaaaagaabaaaabaaaaaajgahbaaaabaaaaaaefaaaaajpcaabaaaadaaaaaa
ogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaadaaaaaadcaaaaapdcaabaaa
adaaaaaahgapbaaaadaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaaapaaaaahicaabaaaabaaaaaa
egaabaaaadaaaaaaegaabaaaadaaaaaaddaaaaahicaabaaaabaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaaabaaaaaadkaabaiaebaaaaaa
abaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaaadaaaaaadkaabaaaabaaaaaa
baaaaaahbcaabaaaabaaaaaaegacbaaaadaaaaaaegacbaaaabaaaaaadeaaaaah
bcaabaaaabaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaa
abaaaaaaakaabaaaabaaaaaadiaaaaahbcaabaaaabaaaaaaakaabaaaabaaaaaa
abeaaaaaaaaaeaecbjaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaagaabaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaaefaaaaaj
pcaabaaaaeaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaacaaaaaa
diaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaaeaaaaaadiaaaaai
hcaabaaaaeaaaaaaegacbaaaaeaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaagiacaaaaaaaaaaaahaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaacaaaaaabaaaaaah
icaabaaaabaaaaaaegacbaaaadaaaaaaegacbaaaacaaaaaabaaaaaahccaabaaa
aaaaaaaajgahbaaaaaaaaaaaegacbaaaadaaaaaaaaaaaaaiccaabaaaaaaaaaaa
bkaabaiaebaaaaaaaaaaaaaadkaabaaaabaaaaaadeaaaaahecaabaaaaaaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaaadcaaaaajhcaabaaaabaaaaaaegacbaaa
aeaaaaaakgakbaaaaaaaaaaaegacbaaaabaaaaaadiaaaaahncaabaaaaaaaaaaa
agaabaaaaaaaaaaaagajbaaaabaaaaaaaaaaaaahccaabaaaaaaaaaaabkaabaaa
aaaaaaaaabeaaaaaaaaaiadpcpaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaa
diaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaaaaaaaaaajaaaaaa
bjaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaaaaaaaaaaiaaaaaadcaaaaajhccabaaaaaaaaaaa
igadbaaaaaaaaaaaegacbaaaabaaaaaaigadbaaaaaaaaaaadgaaaaaficcabaaa
aaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "SPOT" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "SPOT" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
SetTexture 3 [_LightTextureB0] 2D
"agal_ps
c5 2.0 -1.0 1.0 0.0
c6 48.0 0.5 0.0 0.0
[bc]
afaaaaaaacaaabacadaaaappaeaaaaaaaaaaaaaaaaaaaaaa rcp r2.x, v3.w
adaaaaaaacaaadacadaaaaoeaeaaaaaaacaaaaaaacaaaaaa mul r2.xy, v3, r2.x
abaaaaaaacaaadacacaaaafeacaaaaaaagaaaaffabaaaaaa add r2.xy, r2.xyyy, c6.y
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
aaaaaaaaabaaadacaaaaaafeacaaaaaaaaaaaaaaaaaaaaaa mov r1.xy, r0.xyyy
bcaaaaaaaaaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r0.x, v3, v3
aaaaaaaaaaaaadacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.xy, r0.x
ciaaaaaaadaaapacacaaaafeacaaaaaaacaaaaaaafaababb tex r3, r2.xyyy, s2 <2d wrap linear point>
ciaaaaaaabaaapacabaaaafeacaaaaaaabaaaaaaafaababb tex r1, r1.xyyy, s1 <2d wrap linear point>
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
ciaaaaaaaaaaapacaaaaaafeacaaaaaaadaaaaaaafaababb tex r0, r0.xyyy, s3 <2d wrap linear point>
bcaaaaaaadaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r3.x, v2, v2
akaaaaaaadaaabacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r3.x, r3.x
bcaaaaaaabaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, v1, v1
adaaaaaaaeaaahacadaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r4.xyz, r3.x, v2
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaadaaahacabaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r1.x, v1
abaaaaaaagaaahacadaaaakeacaaaaaaaeaaaakeacaaaaaa add r6.xyz, r3.xyzz, r4.xyzz
bcaaaaaaabaaabacagaaaakeacaaaaaaagaaaakeacaaaaaa dp3 r1.x, r6.xyzz, r6.xyzz
aaaaaaaaaaaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r1.y
aaaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.w
adaaaaaaafaaadacaaaaaafeacaaaaaaafaaaaaaabaaaaaa mul r5.xy, r0.xyyy, c5.x
abaaaaaaafaaadacafaaaafeacaaaaaaafaaaaffabaaaaaa add r5.xy, r5.xyyy, c5.y
adaaaaaaaaaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r0.x, r5.y, r5.y
bfaaaaaaaeaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r4.w, r5.x
adaaaaaaaeaaaiacaeaaaappacaaaaaaafaaaaaaacaaaaaa mul r4.w, r4.w, r5.x
acaaaaaaaaaaabacaeaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r4.w, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
adaaaaaaabaaahacabaaaaaaacaaaaaaagaaaakeacaaaaaa mul r1.xyz, r1.x, r6.xyzz
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r1.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
alaaaaaaabaaapacaaaaaaaaacaaaaaaagaaaaaaabaaaaaa pow r1, r0.x, c6.x
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
bcaaaaaaabaaabacaeaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r4.xyzz
adaaaaaaaaaaahacaaaaaaaaacaaaaaaaaaaaaoeabaaaaaa mul r0.xyz, r0.x, c0
adaaaaaaaaaaahacaaaaaakeacaaaaaaacaaaappacaaaaaa mul r0.xyz, r0.xyzz, r2.w
adaaaaaaagaaahacaaaaaakeacaaaaaaacaaaaaaabaaaaaa mul r6.xyz, r0.xyzz, c2.x
bcaaaaaaaaaaabacadaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r0.x, r3.xyzz, r3.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaaeaaaakeacaaaaaa mul r1.xyz, r1.x, r4.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaaeaaahacaaaaaaaaacaaaaaaadaaaakeacaaaaaa mul r4.xyz, r0.x, r3.xyzz
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r1.xyzz
bcaaaaaaadaaabacafaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r3.x, r5.xyzz, r3.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r4.xyzz
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaabaaaaaaacaaaaaa add r0.x, r0.x, r1.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
ahaaaaaaabaaabacadaaaaaaacaaaaaaafaaaappabaaaaaa max r1.x, r3.x, c5.w
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
adaaaaaaabaaahacacaaaakeacaaaaaaabaaaaaaacaaaaaa mul r1.xyz, r2.xyzz, r1.x
adaaaaaaacaaahacagaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r6.xyzz, c1
abaaaaaaacaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa add r2.xyz, r2.xyzz, r1.xyzz
alaaaaaaabaaapacaaaaaaaaacaaaaaaaeaaaaaaabaaaaaa pow r1, r0.x, c4.x
bfaaaaaaaeaaaeacadaaaakkaeaaaaaaaaaaaaaaaaaaaaaa neg r4.z, v3.z
ckaaaaaaaaaaabacaeaaaakkacaaaaaaagaaaakkabaaaaaa slt r0.x, r4.z, c6.z
adaaaaaaaaaaabacaaaaaaaaacaaaaaaadaaaappacaaaaaa mul r0.x, r0.x, r3.w
adaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaappacaaaaaa mul r0.x, r0.x, r0.w
adaaaaaaadaaabacaaaaaaaaacaaaaaaafaaaaoeabaaaaaa mul r3.x, r0.x, c5
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
adaaaaaaabaaahacacaaaakeacaaaaaaadaaaaaaacaaaaaa mul r1.xyz, r2.xyzz, r3.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaadaaaaoeabaaaaaa mul r0.xyz, r0.x, c3
adaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r1.xyzz, r0.xyzz
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaaiacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c5
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "SPOT" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_BumpMap] 2D 3
SetTexture 2 [_LightTexture0] 2D 0
SetTexture 3 [_LightTextureB0] 2D 1
// 50 instructions, 5 temp regs, 0 temp arrays:
// ALU 39 float, 0 int, 1 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_1
eefiecednhknhekhbkoobjinfmnomoecmbkalhfhabaaaaaacealaaaaaeaaaaaa
daaaaaaaoaadaaaafaakaaaapaakaaaaebgpgodjkiadaaaakiadaaaaaaacpppp
fmadaaaaemaaaaaaacaadeaaaaaaemaaaaaaemaaaeaaceaaaaaaemaaacaaaaaa
adababaaaaacacaaabadadaaaaaaabaaacaaaaaaaaaaaaaaaaaaahaaadaaacaa
aaaaaaaaaaacppppfbaaaaafafaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaiadp
fbaaaaafagaaapkaaaaaaadpaaaaeaecaaaaaaaaaaaaaaaabpaaaaacaaaaaaia
aaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaac
aaaaaaiaadaaaplabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapka
bpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkaagaaaaacaaaaaiia
adaapplaaeaaaaaeaaaaadiaadaaoelaaaaappiaagaaaakaaiaaaaadabaaaiia
adaaoelaadaaoelaabaaaaacabaaadiaabaappiaabaaaaacacaaabiaaaaakkla
abaaaaacacaaaciaaaaapplaecaaaaadaaaacpiaaaaaoeiaaaaioekaecaaaaad
abaacpiaabaaoeiaabaioekaecaaaaadacaacpiaacaaoeiaadaioekaecaaaaad
adaaapiaaaaaoelaacaioekaafaaaaadaaaaabiaaaaappiaabaaaaiaacaaaaad
aaaaabiaaaaaaaiaaaaaaaiafiaaaaaeaaaaabiaadaakklbafaakkkaaaaaaaia
aeaaaaaeabaacbiaacaappiaafaaaakaafaaffkaaeaaaaaeabaacciaacaaffia
afaaaakaafaaffkafkaaaaaeabaadiiaabaaoeiaabaaoeiaafaakkkaacaaaaad
abaaciiaabaappibafaappkaahaaaaacabaaciiaabaappiaagaaaaacabaaceia
abaappiaaiaaaaadabaaciiaabaaoelaabaaoelaahaaaaacabaaciiaabaappia
ceaaaaacacaachiaacaaoelaaeaaaaaeaeaachiaabaaoelaabaappiaacaaoeia
afaaaaadafaachiaabaappiaabaaoelaaiaaaaadabaaciiaabaaoeiaafaaoeia
aiaaaaadaeaaciiaacaaoeiaabaaoeiaacaaaaadaeaaciiaabaappiaaeaappib
alaaaaadaaaacciaabaappiaafaakkkaacaaaaadabaaciiaaeaappiaafaappka
caaaaaadaeaaaiiaabaappiaaeaaaakaafaaaaadacaaahiaaeaappiaadaaoeka
ceaaaaacafaachiaaeaaoeiaaiaaaaadacaaciiaabaaoeiaafaaoeiaalaaaaad
aaaaaeiaacaappiaafaakkkacaaaaaadacaaaiiaaaaakkiaagaaffkaafaaaaad
abaaahiaacaappiaaaaaoekaafaaaaadabaaahiaadaappiaabaaoeiaafaaaaad
adaachiaadaaoeiaaaaaoekaafaaaaadabaaahiaabaaoeiaacaaaakaafaaaaad
abaaahiaabaaoeiaabaaoekaaeaaaaaeaaaaaoiaadaabliaaaaaffiaabaablia
afaaaaadaaaachiaaaaaaaiaaaaabliaaeaaaaaeaaaachiaaaaaoeiaacaaoeia
aaaaoeiaabaaaaacaaaaaiiaafaakkkaabaaaaacaaaicpiaaaaaoeiappppaaaa
fdeieefcgiagaaaaeaaaaaaajkabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadpcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaaaoaaaaahdcaabaaaaaaaaaaa
egbabaaaaeaaaaaapgbpbaaaaeaaaaaaaaaaaaakdcaabaaaaaaaaaaaegaabaaa
aaaaaaaaaceaaaaaaaaaaadpaaaaaadpaaaaaaaaaaaaaaaaefaaaaajpcaabaaa
aaaaaaaaegaabaaaaaaaaaaaeghobaaaacaaaaaaaagabaaaaaaaaaaadbaaaaah
bcaabaaaaaaaaaaaabeaaaaaaaaaaaaackbabaaaaeaaaaaaabaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaiadpdiaaaaahbcaabaaaaaaaaaaa
dkaabaaaaaaaaaaaakaabaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaa
aeaaaaaaegbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaafgafbaaaaaaaaaaa
eghobaaaadaaaaaaaagabaaaabaaaaaaapaaaaahbcaabaaaaaaaaaaaagaabaaa
aaaaaaaaagaabaaaabaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaadaaaaaa
egbcbaaaadaaaaaaeeaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaah
ocaabaaaaaaaaaaafgafbaaaaaaaaaaaagbjbaaaadaaaaaabaaaaaahbcaabaaa
abaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaabaaaaaa
akaabaaaabaaaaaadcaaaaajocaabaaaabaaaaaaagbjbaaaacaaaaaaagaabaaa
abaaaaaafgaobaaaaaaaaaaadiaaaaahhcaabaaaacaaaaaaagaabaaaabaaaaaa
egbcbaaaacaaaaaabaaaaaahbcaabaaaabaaaaaajgahbaaaabaaaaaajgahbaaa
abaaaaaaeeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahhcaabaaa
abaaaaaaagaabaaaabaaaaaajgahbaaaabaaaaaaefaaaaajpcaabaaaadaaaaaa
ogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaadaaaaaadcaaaaapdcaabaaa
adaaaaaahgapbaaaadaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaaapaaaaahicaabaaaabaaaaaa
egaabaaaadaaaaaaegaabaaaadaaaaaaddaaaaahicaabaaaabaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaaabaaaaaadkaabaiaebaaaaaa
abaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaaadaaaaaadkaabaaaabaaaaaa
baaaaaahbcaabaaaabaaaaaaegacbaaaadaaaaaaegacbaaaabaaaaaadeaaaaah
bcaabaaaabaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaa
abaaaaaaakaabaaaabaaaaaadiaaaaahbcaabaaaabaaaaaaakaabaaaabaaaaaa
abeaaaaaaaaaeaecbjaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaagaabaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaaefaaaaaj
pcaabaaaaeaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaacaaaaaa
diaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaaeaaaaaadiaaaaai
hcaabaaaaeaaaaaaegacbaaaaeaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaagiacaaaaaaaaaaaahaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaacaaaaaabaaaaaah
icaabaaaabaaaaaaegacbaaaadaaaaaaegacbaaaacaaaaaabaaaaaahccaabaaa
aaaaaaaajgahbaaaaaaaaaaaegacbaaaadaaaaaaaaaaaaaiccaabaaaaaaaaaaa
bkaabaiaebaaaaaaaaaaaaaadkaabaaaabaaaaaadeaaaaahecaabaaaaaaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaaadcaaaaajhcaabaaaabaaaaaaegacbaaa
aeaaaaaakgakbaaaaaaaaaaaegacbaaaabaaaaaadiaaaaahncaabaaaaaaaaaaa
agaabaaaaaaaaaaaagajbaaaabaaaaaaaaaaaaahccaabaaaaaaaaaaabkaabaaa
aaaaaaaaabeaaaaaaaaaiadpcpaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaa
diaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaaaaaaaaaajaaaaaa
bjaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaaaaaaaaaaiaaaaaadcaaaaajhccabaaaaaaaaaaa
igadbaaaaaaaaaaaegacbaaaabaaaaaaigadbaaaaaaaaaaadgaaaaaficcabaaa
aaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaa
iaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaa
imaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaafdfgfpfaepfdejfe
ejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaa
caaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgf
heaaklkl"
}

SubProgram "gles3 " {
Keywords { "SPOT" }
"!!GLES3"
}

SubProgram "opengl " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTextureB0] 2D
SetTexture 3 [_LightTexture0] CUBE
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 49 ALU, 4 TEX
PARAM c[6] = { program.local[0..4],
		{ 0, 2, 1, 48 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R2, fragment.texcoord[0], texture[0], 2D;
TEX R3.yw, fragment.texcoord[0].zwzw, texture[1], 2D;
TEX R1.w, fragment.texcoord[3], texture[3], CUBE;
MAD R4.xy, R3.wyzw, c[5].y, -c[5].z;
DP3 R0.x, fragment.texcoord[3], fragment.texcoord[3];
MUL R4.zw, R4.xyxy, R4.xyxy;
ADD_SAT R3.w, R4.z, R4;
DP3 R1.x, fragment.texcoord[2], fragment.texcoord[2];
RSQ R1.x, R1.x;
ADD R3.w, -R3, c[5].z;
MUL R3.xyz, R1.x, fragment.texcoord[2];
RSQ R3.w, R3.w;
MUL R2.xyz, R2, c[0];
MOV result.color.w, c[5].x;
TEX R0.w, R0.x, texture[2], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.x;
MUL R0.xyz, R0.x, fragment.texcoord[1];
ADD R1.xyz, R0, R3;
DP3 R4.z, R1, R1;
RSQ R4.z, R4.z;
MUL R1.xyz, R4.z, R1;
RCP R4.z, R3.w;
DP3 R1.x, R4, R1;
MAX R1.x, R1, c[5];
POW R1.x, R1.x, c[5].w;
MUL R1.xyz, R1.x, c[0];
MUL R1.xyz, R1, R2.w;
DP3 R3.w, R4, R0;
MAX R2.w, R3, c[5].x;
MUL R2.xyz, R2, R2.w;
MUL R1.xyz, R1, c[2].x;
MAD R1.xyz, R1, c[1], R2;
DP3 R2.y, R0, R0;
RSQ R2.y, R2.y;
MUL R0.xyz, R2.y, R0;
DP3 R2.x, R3, R3;
RSQ R2.x, R2.x;
DP3 R0.y, R4, R0;
MUL R2.xyz, R2.x, R3;
DP3 R0.x, R4, R2;
ADD R0.y, -R0.x, R0;
MUL R0.x, R0.w, R1.w;
ADD R0.y, R0, c[5].z;
POW R0.w, R0.y, c[4].x;
MUL R0.x, R0, c[5].y;
MUL R0.xyz, R1, R0.x;
MUL R1.xyz, R0.w, c[3];
MAD result.color.xyz, R0, R1, R0;
END
# 49 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTextureB0] 2D
SetTexture 3 [_LightTexture0] CUBE
"ps_2_0
; 56 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_cube s3
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 48.00000000, 0, 0, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
texld r2, t3, s3
texld r3, t0, s0
dp3 r0.x, t3, t3
mov r1.xy, r0.x
dp3_pp r2.x, t2, t2
dp3_pp r4.x, t1, t1
rsq_pp r2.x, r2.x
rsq_pp r4.x, r4.x
mul_pp r4.xyz, r4.x, t1
mul_pp r2.xyz, r2.x, t2
add_pp r5.xyz, r4, r2
dp3_pp r6.x, r5, r5
rsq_pp r6.x, r6.x
mul_pp r5.xyz, r6.x, r5
dp3_pp r6.x, r4, r4
mov r0.y, t0.w
mov r0.x, t0.z
rsq_pp r6.x, r6.x
mul_pp r3.xyz, r3, c0
texld r0, r0, s1
texld r1, r1, s2
mov r0.x, r0.w
mad_pp r7.xy, r0, c5.x, c5.y
mul_pp r0.xy, r7, r7
add_pp_sat r0.x, r0, r0.y
add_pp r0.x, -r0, c5.z
rsq_pp r0.x, r0.x
rcp_pp r7.z, r0.x
dp3_pp r0.x, r7, r5
max_pp r0.x, r0, c5.w
pow r5.w, r0.x, c6.x
mov r0.x, r5.w
dp3_pp r5.x, r2, r2
rsq_pp r5.x, r5.x
mul_pp r2.xyz, r5.x, r2
mul_pp r5.xyz, r6.x, r4
mul r0.xyz, r0.x, c0
mul r0.xyz, r0, r3.w
dp3_pp r4.x, r7, r4
max_pp r4.x, r4, c5.w
mul_pp r3.xyz, r3, r4.x
mul r0.xyz, r0, c2.x
mad r0.xyz, r0, c1, r3
dp3_pp r2.x, r7, r2
dp3_pp r5.x, r7, r5
add_pp r2.x, -r2, r5
add_pp r2.x, r2, c5.z
pow_pp r3.x, r2.x, c4.x
mul r1.x, r1, r2.w
mul_pp r1.x, r1, c5
mul r0.xyz, r0, r1.x
mov_pp r2.x, r3.x
mul r1.xyz, r2.x, c3
mad r0.xyz, r0, r1, r0
mov_pp r0.w, c5
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 3 [_RimColor]
Float 4 [_RimPower]
Float 2 [_Shininess]
Vector 1 [_SpecColor]
SetTexture 0 [_LightTexture0] CUBE
SetTexture 1 [_LightTextureB0] 2D
SetTexture 2 [_MainTex] 2D
SetTexture 3 [_BumpMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 45.33 (34 instructions), vertex: 0, texture: 16,
//   sequencer: 16, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~45 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacbiaaaaaceeaaaaaaaaaaaaaaceaaaaabmaaaaaaboiaaaaaaaa
aaaaaaaaaaaaabjiaaaaaabmaaaaabijppppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabicaaaaaanaaaadaaadaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaaa
aaabaaaaaaaaaapmaaaaaaaaaaaaabamaaadaaaaaaabaaaaaaaaabbmaaaaaaaa
aaaaabcmaaadaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabdmaaadaaacaaabaaaa
aaaaaanmaaaaaaaaaaaaabefaaacaaadaaabaaaaaaaaaapmaaaaaaaaaaaaabep
aaacaaaeaaabaaaaaaaaabfmaaaaaaaaaaaaabgmaaacaaacaaabaaaaaaaaabfm
aaaaaaaaaaaaabhhaaacaaabaaabaaaaaaaaaapmaaaaaaaafpechfgnhaengbha
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcda
aaklklklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgihefegfhihehfhc
gfdaaaklaaaeaaaoaaabaaabaaabaaaaaaaaaaaafpemgjghgihefegfhihehfhc
gfecdaaafpengbgjgofegfhiaafpfcgjgnedgpgmgphcaafpfcgjgnfagphhgfhc
aaklklklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpfdgigjgogjgogfhdhdaafp
fdhagfgdedgpgmgphcaahahdfpddfpdaaadccodacodcdadddfddcodaaaklklkl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaacaebaaaagaaaaaaaaaeaaaaaaaaaaaadeie
aaapaaapaaaaaaabaaaapafaaaaahbfbaaaahcfcaaaahdfdaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaadpmaaaaa
aaaaaaaaaaaaaaaaeaaaaaaaaaaaaaaalpiaaaaaeceaaaaaafaagaaecaakbcaa
bcaaaaafaaaaaaaagaammeaabcaaaaaaaaaagabcgabibcaabcaaaaaaaaaagabo
gacebcaaccaaaaaamiaiaaabaaloloaapaadadaamiapaaadaakgmnaapcadadaa
emieacaeaablblmgocadadidmiadaaaeaagnbllbmladacpobacidaabbpbppgii
aaaaeaaajaaicaibbpbpphppaaaamaaapmbibacbbpbppbppaaaaeaaadidiaaab
bpbpphpjaaaaeaaamiacaaaaaaloloaapaababaamiaeaaaaaaloloaapaacacaa
miadaaaeaagpgmmgilaappppfiebaaaaaablblmgobacabiafichaaacaamgmalb
obaaaciamiahaaafaalbmaaaobaaabaamjacaaaaaalalalbnbaeaeppmiaiaaaa
aaloloaapaafafaamianaaabaapapaaaoaafacaaliecaaabaaloloebnaacacpo
ficcabaaaampmplbpaababibfiihaaacaamalbblobacabiakaehaeagaamablmg
obafaaiamiaiaaaaaaloloaapaagaeaaficcaaabaalololbpaafaeiamianaaab
aaaelbaaobabaaaamiacaaaaaaloloaapaacaeaamiabaaabaamploaapaabaeaa
liekaaabaalmlbebicabpppoeabfaaabaagogplboaaaaaibeacbaaaaaagmblgm
kbaappibdibcaaaaaalbgmgmkbaaaeaadiihaaacaamamalbkbadaaaamiahaaaa
aagmmaaakbaaaaaamiahaaaaaamablaaobaaadaakibhabaaaamagmedibaaacad
kichabaaaamamaidibaaabadmiahaaaaaamablmaolacabaakiehabaaaamgmamd
mbabaaadmiahmaaaaamamamaolaaabaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTextureB0] 2D
SetTexture 3 [_LightTexture0] CUBE
"sce_fp_rsx // 55 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840004000000
[Offsets]
5
_LightColor0 2 0
00000290000000c0
_SpecColor 1 0
000002b0
_Shininess 1 0
00000240
_RimColor 1 0
00000330
_RimPower 1 0
000002e0
[Microcode]
880
ee040100c8011c9dc8000001c8003fe110040500c8081c9dc8080001c8000001
940017025c011c9dc8000001c8003fe1068e0440ce001c9d00020000aa020000
000040000000bf800000000000000000ae8c3940c8011c9dc8000029c800bfe1
0280b840c91c1c9dc91c0001c80000019e021700c8011c9dc8000001c8003fe1
ce823940c8011c9dc8000029c800bfe11080034001001c9ec8020001c8000001
00000000000000000000000000003f800e840240c8041c9dc8020001c8000001
00000000000000000000000000000000088e3b40ff003c9dff000001c8000001
06060100c80c1c9dc8000001c80000010e040340c9181c9dc9040001c8000001
02001704fe081c9dc8000001c80000010e883940c8081c9dc8000029c8000001
08800540c91c1c9dc9100001c80000010400090055001c9d00020000c8000001
0000000000000000000000000000000004001d00aa001c9cc8000001c8000001
108c0540c91c1c9dc9180001c800000110840900c9181c9d00020000c8000001
000000000000000000000000000000000e840240c9081c9dff080001c8000001
10040200aa001c9c00020000c800000100004240000000000000000000000000
0e823940c9041c9dc8000029c800000108800540c91c1c9dc9040001c8000001
08021c00fe081c9dc8000001c80000010e823940c9181c9dc8000029c8000001
10800540c91c1c9dc9040001c80000011002020054041c9dc8040001c8000001
f0001706c8011c9dc8000001c8003fe108000200fe041c9d00020000c8000001
000000000000000000000000000000000286034055001c9fff000001c8000001
10800340010c1c9cc8020001c800000100000000000000000000000000003f80
0e04020054001c9dc8020001c800000100000000000000000000000000000000
0e040400c8081c9dc8020001c908000100000000000000000000000000000000
04001d00ff001c9dc8000001c800000110020200aa001c9c00020000c8000001
0000000000000000000000000000000002800200c8001c9dfe001001c8000001
0e020200c8081c9d01000000c800000110021c00fe041c9dc8000001c8000001
16000200fe041c9d48020001c800000100000000000000000000000000000000
0e800400c8041c9d68000001c80400011081014000021c9cc8000001c8000001
00000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "POINT_COOKIE" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_BumpMap] 2D 3
SetTexture 2 [_LightTextureB0] 2D 1
SetTexture 3 [_LightTexture0] CUBE 0
// 45 instructions, 5 temp regs, 0 temp arrays:
// ALU 35 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedhkhcfelecniakakendeggipielkhhefnabaaaaaaniagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcnaafaaaaeaaaaaaaheabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fidaaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegbcbaaaadaaaaaa
baaaaaahicaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaf
icaabaaaaaaaaaaadkaabaaaaaaaaaaadcaaaaajhcaabaaaabaaaaaaegbcbaaa
acaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadiaaaaahhcaabaaaacaaaaaa
pgapbaaaaaaaaaaaegbcbaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaabaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaa
diaaaaahhcaabaaaabaaaaaapgapbaaaaaaaaaaaegacbaaaabaaaaaaefaaaaaj
pcaabaaaadaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaadaaaaaa
dcaaaaapdcaabaaaadaaaaaahgapbaaaadaaaaaaaceaaaaaaaaaaaeaaaaaaaea
aaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaaapaaaaah
icaabaaaaaaaaaaaegaabaaaadaaaaaaegaabaaaadaaaaaaddaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaaaaaaaaaa
dkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaaadaaaaaa
dkaabaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaadaaaaaaegacbaaa
abaaaaaadeaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaa
cpaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahicaabaaaaaaaaaaa
dkaabaaaaaaaaaaaabeaaaaaaaaaeaecbjaaaaaficaabaaaaaaaaaaadkaabaaa
aaaaaaaadiaaaaaihcaabaaaabaaaaaapgapbaaaaaaaaaaaegiccaaaaaaaaaaa
abaaaaaaefaaaaajpcaabaaaaeaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaa
aagabaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaa
aeaaaaaadiaaaaaihcaabaaaaeaaaaaaegacbaaaaeaaaaaaegiccaaaaaaaaaaa
abaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaagiacaaaaaaaaaaa
ahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaa
acaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaadaaaaaaegacbaaaacaaaaaa
baaaaaahbcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaadaaaaaaaaaaaaai
bcaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpaaaaaaah
bcaabaaaaaaaaaaadkaabaaaaaaaaaaaakaabaaaaaaaaaaadeaaaaahccaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaa
agajbaaaaeaaaaaafgafbaaaaaaaaaaaagajbaaaabaaaaaacpaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaaibcaabaaaaaaaaaaaakaabaaaaaaaaaaa
akiacaaaaaaaaaaaajaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaaihcaabaaaabaaaaaaagaabaaaaaaaaaaaegiccaaaaaaaaaaaaiaaaaaa
baaaaaahbcaabaaaaaaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaaefaaaaaj
pcaabaaaacaaaaaaagaabaaaaaaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaa
efaaaaajpcaabaaaadaaaaaaegbcbaaaaeaaaaaaeghobaaaadaaaaaaaagabaaa
aaaaaaaaapaaaaahbcaabaaaaaaaaaaaagaabaaaacaaaaaapgapbaaaadaaaaaa
diaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaadcaaaaaj
hccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "POINT_COOKIE" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "POINT_COOKIE" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTextureB0] 2D
SetTexture 3 [_LightTexture0] CUBE
"agal_ps
c5 2.0 -1.0 1.0 0.0
c6 48.0 0.0 0.0 0.0
[bc]
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
bcaaaaaaaaaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r0.x, v3, v3
aaaaaaaaaaaaadacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.xy, r0.x
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
aaaaaaaaabaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r1.y, v0.w
aaaaaaaaabaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r1.x, v0.z
ciaaaaaaadaaapacaaaaaafeacaaaaaaacaaaaaaafaababb tex r3, r0.xyyy, s2 <2d wrap linear point>
ciaaaaaaabaaapacabaaaafeacaaaaaaabaaaaaaafaababb tex r1, r1.xyyy, s1 <2d wrap linear point>
ciaaaaaaaaaaapacadaaaaoeaeaaaaaaadaaaaaaafbababb tex r0, v3, s3 <cube wrap linear point>
bcaaaaaaadaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r3.x, v2, v2
akaaaaaaadaaabacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r3.x, r3.x
bcaaaaaaabaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, v1, v1
adaaaaaaaeaaahacadaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r4.xyz, r3.x, v2
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaadaaahacabaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r1.x, v1
abaaaaaaagaaahacadaaaakeacaaaaaaaeaaaakeacaaaaaa add r6.xyz, r3.xyzz, r4.xyzz
bcaaaaaaabaaabacagaaaakeacaaaaaaagaaaakeacaaaaaa dp3 r1.x, r6.xyzz, r6.xyzz
aaaaaaaaaaaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r1.y
aaaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.w
adaaaaaaafaaadacaaaaaafeacaaaaaaafaaaaaaabaaaaaa mul r5.xy, r0.xyyy, c5.x
abaaaaaaafaaadacafaaaafeacaaaaaaafaaaaffabaaaaaa add r5.xy, r5.xyyy, c5.y
adaaaaaaaaaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r0.x, r5.y, r5.y
bfaaaaaaaeaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r4.w, r5.x
adaaaaaaaeaaaiacaeaaaappacaaaaaaafaaaaaaacaaaaaa mul r4.w, r4.w, r5.x
acaaaaaaaaaaabacaeaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r4.w, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
adaaaaaaabaaahacabaaaaaaacaaaaaaagaaaakeacaaaaaa mul r1.xyz, r1.x, r6.xyzz
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r1.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
alaaaaaaabaaapacaaaaaaaaacaaaaaaagaaaaaaabaaaaaa pow r1, r0.x, c6.x
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
bcaaaaaaabaaabacaeaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r4.xyzz
adaaaaaaaaaaahacaaaaaaaaacaaaaaaaaaaaaoeabaaaaaa mul r0.xyz, r0.x, c0
adaaaaaaaaaaahacaaaaaakeacaaaaaaacaaaappacaaaaaa mul r0.xyz, r0.xyzz, r2.w
adaaaaaaagaaahacaaaaaakeacaaaaaaacaaaaaaabaaaaaa mul r6.xyz, r0.xyzz, c2.x
bcaaaaaaaaaaabacadaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r0.x, r3.xyzz, r3.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaaeaaaakeacaaaaaa mul r1.xyz, r1.x, r4.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaaeaaahacaaaaaaaaacaaaaaaadaaaakeacaaaaaa mul r4.xyz, r0.x, r3.xyzz
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r1.xyzz
bcaaaaaaadaaabacafaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r3.x, r5.xyzz, r3.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r4.xyzz
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaabaaaaaaacaaaaaa add r0.x, r0.x, r1.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
ahaaaaaaabaaabacadaaaaaaacaaaaaaafaaaappabaaaaaa max r1.x, r3.x, c5.w
adaaaaaaabaaahacacaaaakeacaaaaaaabaaaaaaacaaaaaa mul r1.xyz, r2.xyzz, r1.x
adaaaaaaacaaahacagaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r6.xyzz, c1
abaaaaaaacaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa add r2.xyz, r2.xyzz, r1.xyzz
alaaaaaaabaaapacaaaaaaaaacaaaaaaaeaaaaaaabaaaaaa pow r1, r0.x, c4.x
adaaaaaaaaaaabacadaaaappacaaaaaaaaaaaappacaaaaaa mul r0.x, r3.w, r0.w
adaaaaaaadaaabacaaaaaaaaacaaaaaaafaaaaoeabaaaaaa mul r3.x, r0.x, c5
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
adaaaaaaabaaahacacaaaakeacaaaaaaadaaaaaaacaaaaaa mul r1.xyz, r2.xyzz, r3.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaadaaaaoeabaaaaaa mul r0.xyz, r0.x, c3
adaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r1.xyzz, r0.xyzz
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaaiacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c5
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT_COOKIE" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_BumpMap] 2D 3
SetTexture 2 [_LightTextureB0] 2D 1
SetTexture 3 [_LightTexture0] CUBE 0
// 45 instructions, 5 temp regs, 0 temp arrays:
// ALU 35 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_1
eefiecedelklchgaolieponidplfpbmfahanhkfbabaaaaaafiakaaaaaeaaaaaa
daaaaaaakmadaaaaieajaaaaceakaaaaebgpgodjheadaaaaheadaaaaaaacpppp
ciadaaaaemaaaaaaacaadeaaaaaaemaaaaaaemaaaeaaceaaaaaaemaaadaaaaaa
acababaaaaacacaaabadadaaaaaaabaaacaaaaaaaaaaaaaaaaaaahaaadaaacaa
aaaaaaaaaaacppppfbaaaaafafaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaiadp
fbaaaaafagaaapkaaaaaeaecaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaia
aaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaac
aaaaaaiaadaaahlabpaaaaacaaaaaajiaaaiapkabpaaaaacaaaaaajaabaiapka
bpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkaaiaaaaadaaaaaiia
adaaoelaadaaoelaabaaaaacaaaaadiaaaaappiaabaaaaacabaaabiaaaaakkla
abaaaaacabaaaciaaaaapplaecaaaaadaaaaapiaaaaaoeiaabaioekaecaaaaad
acaaapiaadaaoelaaaaioekaecaaaaadabaacpiaabaaoeiaadaioekaecaaaaad
adaaapiaaaaaoelaacaioekaafaaaaadaaaacbiaaaaaaaiaacaappiaacaaaaad
aaaaabiaaaaaaaiaaaaaaaiaaeaaaaaeacaacbiaabaappiaafaaaakaafaaffka
aeaaaaaeacaacciaabaaffiaafaaaakaafaaffkafkaaaaaeacaadiiaacaaoeia
acaaoeiaafaakkkaacaaaaadacaaciiaacaappibafaappkaahaaaaacacaaciia
acaappiaagaaaaacacaaceiaacaappiaaiaaaaadacaaciiaabaaoelaabaaoela
ahaaaaacacaaciiaacaappiaceaaaaacabaachiaacaaoelaaeaaaaaeaeaachia
abaaoelaacaappiaabaaoeiaafaaaaadafaachiaacaappiaabaaoelaaiaaaaad
abaaciiaacaaoeiaafaaoeiaaiaaaaadacaaciiaabaaoeiaacaaoeiaacaaaaad
acaaciiaabaappiaacaappibalaaaaadaeaaciiaabaappiaafaakkkaacaaaaad
acaaciiaacaappiaafaappkacaaaaaadaaaaaciaacaappiaaeaaaakaafaaaaad
aaaaaoiaaaaaffiaadaablkaceaaaaacabaachiaaeaaoeiaaiaaaaadabaacbia
acaaoeiaabaaoeiaalaaaaadacaaabiaabaaaaiaafaakkkacaaaaaadabaaabia
acaaaaiaagaaaakaafaaaaadabaaahiaabaaaaiaaaaaoekaafaaaaadabaaahia
adaappiaabaaoeiaafaaaaadacaachiaadaaoeiaaaaaoekaafaaaaadabaaahia
abaaoeiaacaaaakaafaaaaadabaaahiaabaaoeiaabaaoekaaeaaaaaeabaaahia
acaaoeiaaeaappiaabaaoeiaafaaaaadabaachiaaaaaaaiaabaaoeiaaeaaaaae
aaaachiaabaaoeiaaaaabliaabaaoeiaabaaaaacaaaaciiaafaakkkaabaaaaac
aaaicpiaaaaaoeiappppaaaafdeieefcnaafaaaaeaaaaaaaheabaaaafjaaaaae
egiocaaaaaaaaaaaakaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaa
abaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafidaaaaeaahabaaaadaaaaaaffffaaaagcbaaaad
pcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gcbaaaadhcbabaaaaeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaa
baaaaaahbcaabaaaaaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaa
aaaaaaaaegbcbaaaadaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaacaaaaaa
egbcbaaaacaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadcaaaaaj
hcaabaaaabaaaaaaegbcbaaaacaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
diaaaaahhcaabaaaacaaaaaapgapbaaaaaaaaaaaegbcbaaaacaaaaaabaaaaaah
icaabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaabaaaaaaeeaaaaaficaabaaa
aaaaaaaadkaabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaapgapbaaaaaaaaaaa
egacbaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaogbkbaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaadaaaaaadcaaaaapdcaabaaaadaaaaaahgapbaaaadaaaaaa
aceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialp
aaaaaaaaaaaaaaaaapaaaaahicaabaaaaaaaaaaaegaabaaaadaaaaaaegaabaaa
adaaaaaaddaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadp
aaaaaaaiicaabaaaaaaaaaaadkaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadp
elaaaaafecaabaaaadaaaaaadkaabaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaa
egacbaaaadaaaaaaegacbaaaabaaaaaadeaaaaahicaabaaaaaaaaaaadkaabaaa
aaaaaaaaabeaaaaaaaaaaaaacpaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaa
diaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaeaecbjaaaaaf
icaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaapgapbaaa
aaaaaaaaegiccaaaaaaaaaaaabaaaaaaefaaaaajpcaabaaaaeaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaapgapbaaaaeaaaaaadiaaaaaihcaabaaaaeaaaaaaegacbaaa
aeaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaagiacaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaa
adaaaaaaegacbaaaacaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaadaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaa
abeaaaaaaaaaiadpaaaaaaahbcaabaaaaaaaaaaadkaabaaaaaaaaaaaakaabaaa
aaaaaaaadeaaaaahccaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaa
dcaaaaajocaabaaaaaaaaaaaagajbaaaaeaaaaaafgafbaaaaaaaaaaaagajbaaa
abaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaibcaabaaa
aaaaaaaaakaabaaaaaaaaaaaakiacaaaaaaaaaaaajaaaaaabjaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaaagaabaaaaaaaaaaa
egiccaaaaaaaaaaaaiaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaaeaaaaaa
egbcbaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaagaabaaaaaaaaaaaeghobaaa
acaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaegbcbaaaaeaaaaaa
eghobaaaadaaaaaaaagabaaaaaaaaaaaapaaaaahbcaabaaaaaaaaaaaagaabaaa
acaaaaaapgapbaaaadaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaa
jgahbaaaaaaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaa
doaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaa
adaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaa
apapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaaadaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "gles3 " {
Keywords { "POINT_COOKIE" }
"!!GLES3"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 43 ALU, 3 TEX
PARAM c[6] = { program.local[0..4],
		{ 0, 2, 1, 48 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R1.yw, fragment.texcoord[0].zwzw, texture[1], 2D;
TEX R2.w, fragment.texcoord[3], texture[2], 2D;
DP3 R1.x, fragment.texcoord[2], fragment.texcoord[2];
RSQ R1.z, R1.x;
MUL R2.xyz, R1.z, fragment.texcoord[2];
MAD R1.xy, R1.wyzw, c[5].y, -c[5].z;
MUL R1.zw, R1.xyxy, R1.xyxy;
ADD_SAT R1.z, R1, R1.w;
ADD R3.xyz, fragment.texcoord[1], R2;
DP3 R1.w, R3, R3;
RSQ R1.w, R1.w;
ADD R1.z, -R1, c[5];
RSQ R1.z, R1.z;
MUL R3.xyz, R1.w, R3;
RCP R1.z, R1.z;
DP3 R1.w, R1, R3;
DP3 R3.x, R2, R2;
RSQ R3.w, R3.x;
MUL R2.xyz, R3.w, R2;
MAX R1.w, R1, c[5].x;
DP3 R3.y, fragment.texcoord[1], fragment.texcoord[1];
RSQ R3.y, R3.y;
MUL R3.xyz, R3.y, fragment.texcoord[1];
DP3 R2.x, R1, R2;
DP3 R3.x, R1, R3;
POW R1.w, R1.w, c[5].w;
ADD R3.x, -R2, R3;
MUL R2.xyz, R1.w, c[0];
DP3 R1.w, R1, fragment.texcoord[1];
MUL R1.xyz, R2, R0.w;
MUL R1.xyz, R1, c[2].x;
MAX R0.w, R1, c[5].x;
MUL R0.xyz, R0, c[0];
MUL R0.xyz, R0, R0.w;
MAD R0.xyz, R1, c[1], R0;
ADD R1.x, R3, c[5].z;
MUL R0.w, R2, c[5].y;
POW R1.x, R1.x, c[4].x;
MUL R0.xyz, R0, R0.w;
MUL R1.xyz, R1.x, c[3];
MAD result.color.xyz, R0, R1, R0;
MOV result.color.w, c[5].x;
END
# 43 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
"ps_2_0
; 51 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 48.00000000, 0, 0, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xy
texld r2, t0, s0
mov r0.y, t0.w
mov r0.x, t0.z
mov r1.xy, r0
mul_pp r2.xyz, r2, c0
texld r1, r1, s1
texld r0, t3, s2
mov r1.x, r1.w
dp3_pp r0.x, t2, t2
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t2
mad_pp r4.xy, r1, c5.x, c5.y
add_pp r5.xyz, t1, r3
mul_pp r0.xy, r4, r4
add_pp_sat r0.x, r0, r0.y
dp3_pp r1.x, r5, r5
add_pp r0.x, -r0, c5.z
rsq_pp r0.x, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r5
rcp_pp r4.z, r0.x
dp3_pp r0.x, r4, r1
max_pp r0.x, r0, c5.w
pow r5.x, r0.x, c6.x
dp3_pp r0.x, r3, r3
rsq_pp r0.x, r0.x
dp3_pp r1.x, t1, t1
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, t1
mul_pp r3.xyz, r0.x, r3
dp3_pp r0.x, r4, r1
dp3_pp r1.x, r4, r3
add_pp r0.x, -r1, r0
add_pp r0.x, r0, c5.z
mov r1.x, r5.x
mul r3.xyz, r1.x, c0
dp3_pp r1.x, r4, t1
max_pp r1.x, r1, c5.w
mul_pp r1.xyz, r2, r1.x
mul r3.xyz, r3, r2.w
mul r2.xyz, r3, c2.x
mad r2.xyz, r2, c1, r1
pow_pp r1.w, r0.x, c4.x
mul_pp r3.x, r0.w, c5
mov_pp r0.x, r1.w
mul r1.xyz, r2, r3.x
mul r0.xyz, r0.x, c3
mad r0.xyz, r1, r0, r1
mov_pp r0.w, c5
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 3 [_RimColor]
Float 4 [_RimPower]
Float 2 [_Shininess]
Vector 1 [_SpecColor]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 36.00 (27 instructions), vertex: 0, texture: 12,
//   sequencer: 14, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~36 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaaboaaaaaaboeaaaaaaaaaaaaaaceaaaaabiiaaaaablaaaaaaaaa
aaaaaaaaaaaaabgaaaaaaabmaaaaabfbppppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabekaaaaaalmaaadaaacaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaaa
aaabaaaaaaaaaaoiaaaaaaaaaaaaaapiaaadaaaaaaabaaaaaaaaaamiaaaaaaaa
aaaaabahaaadaaabaaabaaaaaaaaaamiaaaaaaaaaaaaabbaaaacaaadaaabaaaa
aaaaaaoiaaaaaaaaaaaaabbkaaacaaaeaaabaaaaaaaaabceaaaaaaaaaaaaabde
aaacaaacaaabaaaaaaaaabceaaaaaaaaaaaaabdpaaacaaabaaabaaaaaaaaaaoi
aaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpemgjghgiheedgpgmgphcdaaaklklklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpemgjghgihefegfhihehfhcgfdaaafpengbgjgofegfhiaafpfcgjgnedgpgmgp
hcaafpfcgjgnfagphhgfhcaaaaaaaaadaaabaaabaaabaaaaaaaaaaaafpfdgigj
gogjgogfhdhdaafpfdhagfgdedgpgmgphcaahahdfpddfpdaaadccodacodcdadd
dfddcodaaaklklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaaba
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabkebaaaagaaaaaaaaae
aaaaaaaaaaaadaieaaapaaapaaaaaaabaaaapafaaaaahbfbaaaahcfcaaaaddfd
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
eaaaaaaaaaaaaaaalpiaaaaaeceaaaaadpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aabfdaaeaaaabcaameaaaaaaaaaagaahgaanbcaabcaaaaaaaaaagabdgabjbcaa
bcaaaaaaaaaadabpaaaaccaaaaaaaaaababieaabbpbppgiiaaaaeaaabaaidagb
bpbpppnpaaaaeaaadiciaaabbpbppopjaaaaeaaamiacaaaaaaloloaapaababaa
miadaaafaagogmmgilaapopoficbaaaaaalololbpaacaciafibnaaadaalbpagm
obaaabiamiahaaagaagmmaaaobaaacaamjacaaaaaalalalbnbafafpomiahaaac
aamamaaaoaagabaalibeaaaaaaloloebnaagagppficiaaabaalolomgpaacacia
kaeoafaaaapmlbgmobagaaiamiaeaaadaamploaapaadafaafibbabaaaalolobl
paafabibmiahaaabaamagmaaobacabaamiacaaaaaamdloaapaaaafaamiaeaaaa
aaloloaapaabafaalibkadabaagglbebicaapoppeabfaaabaalalclboaadadib
eacbaaaaaagmblgmkbaapoibdibcaaaaaalbgmgmkbaaaeaadiihaaacaamamalb
kbaeaaaamiahaaaaaagmmaaakbaaaaaamiahaaaaaamablaaobaaaeaakibhabaa
aamagmedibaaacadkichabaaaamamaidibaaabadmiahaaaaaamablmaolacabaa
kiehabaaaamgmamdmbabaaadmiahmaaaaamamamaolaaabaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
"sce_fp_rsx // 50 instructions using 3 registers
[Configuration]
24
ffffffff0003c020000ffff8000000000000840003000000
[Offsets]
5
_LightColor0 2 0
0000022000000120
_SpecColor 1 0
00000240
_Shininess 1 0
000001f0
_RimColor 1 0
000002e0
_RimPower 1 0
000002b0
[Microcode]
800
940017025c011c9dc8000001c8003fe106820440ce001c9d00020000aa020000
000040000000bf800000000000000000ce803940c8011c9dc8000029c800bfe1
1080b840c9041c9dc9040001c800000110800340c9001c9f00020000c8000001
00003f80000000000000000000000000ae880140c8011c9dc8000001c8003fe1
0e8a0340c9101c9dc9000001c800000108823b40ff003c9dff000001c8000001
0e843940c9141c9dc8000029c800000110800540c9041c9dc9080001c8000001
0e803940c9001c9dc8000029c800000110040900c9001c9d00020000c8000001
0000000000000000000000000000000010800540c9041c9dc9000001c8000001
9e021700c8011c9dc8000001c8003fe10e840240c8041c9dc8020001c8000001
0000000000000000000000000000000002001d00fe081c9dc8000001c8000001
1004020000001c9c00020000c800000100004240000000000000000000000000
10840540c9041c9dc9100001c800000108021c00fe081c9dc8000001c8000001
ae803940c8011c9dc8000029c8003fe110840900c9081c9d00020000c8000001
0000000000000000000000000000000002800540c9041c9dc9000001c8000001
0e840240c9081c9dff080001c80000011002020054041c9dc8040001c8000001
10020200c8041c9d00020000c800000100000000000000000000000000000000
10820340c9001c9f01000000c80000010e000200fe041c9dc8020001c8000001
000000000000000000000000000000000e000400c8001c9dc8021001c9080001
0000000000000000000000000000000010820340c9041c9dc8020001c8000001
00000000000000000000000000003f80f0041704c8011c9dc8000001c8003fe1
0e000200c8001c9dfe080001c800000110001d00ff041c9dc8000001c8000001
10000200c8001c9d00020000c800000100000000000000000000000000000000
10001c00fe001c9dc8000001c800000116020200fe001c9d48020001c8000001
000000000000000000000000000000000e800400c8001c9d68040001c8000001
1081014000021c9cc8000001c800000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL_COOKIE" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_BumpMap] 2D 2
SetTexture 2 [_LightTexture0] 2D 0
// 44 instructions, 4 temp regs, 0 temp arrays:
// ALU 35 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedfoghahjjjiimhagcjnahoidnaffhplkaabaaaaaajiagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadadaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcjaafaaaaeaaaaaaageabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaa
ffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaa
gcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaaddcbabaaa
aeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaaagbjbaaaadaaaaaaagaabaaa
aaaaaaaaagbjbaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaa
egbcbaaaadaaaaaabaaaaaahbcaabaaaaaaaaaaajgahbaaaaaaaaaaajgahbaaa
aaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaa
ogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaadcaaaaapdcaabaaa
acaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaaapaaaaahicaabaaaaaaaaaaa
egaabaaaacaaaaaaegaabaaaacaaaaaaddaaaaahicaabaaaaaaaaaaadkaabaaa
aaaaaaaaabeaaaaaaaaaiadpaaaaaaaiicaabaaaaaaaaaaadkaabaiaebaaaaaa
aaaaaaaaabeaaaaaaaaaiadpelaaaaafecaabaaaacaaaaaadkaabaaaaaaaaaaa
baaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaaaaaaaaadeaaaaah
bcaabaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
abeaaaaaaaaaeaecbjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaai
hcaabaaaaaaaaaaaagaabaaaaaaaaaaaegiccaaaaaaaaaaaabaaaaaaefaaaaaj
pcaabaaaadaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaadaaaaaadiaaaaai
hcaabaaaadaaaaaaegacbaaaadaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaai
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaagiacaaaaaaaaaaaahaaaaaadiaaaaai
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaacaaaaaabaaaaaah
icaabaaaaaaaaaaaegacbaaaacaaaaaaegbcbaaaacaaaaaadeaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaadaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaa
adaaaaaaegbabaaaaeaaaaaaeghobaaaacaaaaaaaagabaaaaaaaaaaaaaaaaaah
icaabaaaaaaaaaaadkaabaaaadaaaaaadkaabaaaadaaaaaadiaaaaahhcaabaaa
aaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaa
egacbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaaiicaabaaaaaaaaaaadkaabaia
ebaaaaaaaaaaaaaaabeaaaaaaaaaiadpbaaaaaahbcaabaaaabaaaaaaegbcbaaa
acaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaa
diaaaaahhcaabaaaabaaaaaaagaabaaaabaaaaaaegbcbaaaacaaaaaabaaaaaah
bcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaakaabaaaabaaaaaacpaaaaaficaabaaaaaaaaaaa
dkaabaaaaaaaaaaadiaaaaaiicaabaaaaaaaaaaadkaabaaaaaaaaaaaakiacaaa
aaaaaaaaajaaaaaabjaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaai
hcaabaaaabaaaaaapgapbaaaaaaaaaaaegiccaaaaaaaaaaaaiaaaaaadcaaaaaj
hccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_SpecColor]
Float 2 [_Shininess]
Vector 3 [_RimColor]
Float 4 [_RimPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_LightTexture0] 2D
"agal_ps
c5 2.0 -1.0 1.0 0.0
c6 48.0 0.0 0.0 0.0
[bc]
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
aaaaaaaaabaaadacaaaaaafeacaaaaaaaaaaaaaaaaaaaaaa mov r1.xy, r0.xyyy
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
ciaaaaaaabaaapacabaaaafeacaaaaaaabaaaaaaafaababb tex r1, r1.xyyy, s1 <2d wrap linear point>
ciaaaaaaaaaaapacadaaaaoeaeaaaaaaacaaaaaaafaababb tex r0, v3, s2 <2d wrap linear point>
aaaaaaaaabaaabacabaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r1.w
bcaaaaaaaaaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, v2, v2
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r3.xyz, r0.x, v2
adaaaaaaaeaaadacabaaaafeacaaaaaaafaaaaaaabaaaaaa mul r4.xy, r1.xyyy, c5.x
abaaaaaaaeaaadacaeaaaafeacaaaaaaafaaaaffabaaaaaa add r4.xy, r4.xyyy, c5.y
abaaaaaaafaaahacabaaaaoeaeaaaaaaadaaaakeacaaaaaa add r5.xyz, v1, r3.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r5.xyzz
adaaaaaaaaaaabacaeaaaaffacaaaaaaaeaaaaffacaaaaaa mul r0.x, r4.y, r4.y
bfaaaaaaadaaaiacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r3.w, r4.x
adaaaaaaadaaaiacadaaaappacaaaaaaaeaaaaaaacaaaaaa mul r3.w, r3.w, r4.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r3.w, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaafaaaakeacaaaaaa mul r1.xyz, r1.x, r5.xyzz
afaaaaaaaeaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r4.z, r0.x
bcaaaaaaaaaaabacaeaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r4.xyzz, r1.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaappabaaaaaa max r0.x, r0.x, c5.w
alaaaaaaafaaapacaaaaaaaaacaaaaaaagaaaaaaabaaaaaa pow r5, r0.x, c6.x
bcaaaaaaaaaaabacadaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r0.x, r3.xyzz, r3.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
bcaaaaaaabaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, v1, v1
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r1.xyz, r1.x, v1
adaaaaaaadaaahacaaaaaaaaacaaaaaaadaaaakeacaaaaaa mul r3.xyz, r0.x, r3.xyzz
bcaaaaaaaaaaabacaeaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r0.x, r4.xyzz, r1.xyzz
bcaaaaaaabaaabacaeaaaakeacaaaaaaadaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r3.xyzz
bfaaaaaaaeaaaiacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r4.w, r1.x
abaaaaaaaaaaabacaeaaaappacaaaaaaaaaaaaaaacaaaaaa add r0.x, r4.w, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaakkabaaaaaa add r0.x, r0.x, c5.z
aaaaaaaaabaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r5.x
adaaaaaaadaaahacabaaaaaaacaaaaaaaaaaaaoeabaaaaaa mul r3.xyz, r1.x, c0
bcaaaaaaabaaabacaeaaaakeacaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, r4.xyzz, v1
ahaaaaaaabaaabacabaaaaaaacaaaaaaafaaaappabaaaaaa max r1.x, r1.x, c5.w
adaaaaaaabaaahacacaaaakeacaaaaaaabaaaaaaacaaaaaa mul r1.xyz, r2.xyzz, r1.x
adaaaaaaadaaahacadaaaakeacaaaaaaacaaaappacaaaaaa mul r3.xyz, r3.xyzz, r2.w
adaaaaaaacaaahacadaaaakeacaaaaaaacaaaaaaabaaaaaa mul r2.xyz, r3.xyzz, c2.x
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
abaaaaaaacaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa add r2.xyz, r2.xyzz, r1.xyzz
alaaaaaaabaaapacaaaaaaaaacaaaaaaaeaaaaaaabaaaaaa pow r1, r0.x, c4.x
adaaaaaaadaaabacaaaaaappacaaaaaaafaaaaoeabaaaaaa mul r3.x, r0.w, c5
aaaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.x
adaaaaaaabaaahacacaaaakeacaaaaaaadaaaaaaacaaaaaa mul r1.xyz, r2.xyzz, r3.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaadaaaaoeabaaaaaa mul r0.xyz, r0.x, c3
adaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r1.xyzz, r0.xyzz
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaaiacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c5
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL_COOKIE" }
ConstBuffer "$Globals" 192 // 148 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 32 [_SpecColor] 4
Float 112 [_Shininess]
Vector 128 [_RimColor] 4
Float 144 [_RimPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_BumpMap] 2D 2
SetTexture 2 [_LightTexture0] 2D 0
// 44 instructions, 4 temp regs, 0 temp arrays:
// ALU 35 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_1
eefieceddgceipiegkfnfggbbcmdlimdmchimkogabaaaaaaoiajaaaaaeaaaaaa
daaaaaaahmadaaaabeajaaaaleajaaaaebgpgodjeeadaaaaeeadaaaaaaacpppp
pmacaaaaeiaaaaaaacaadaaaaaaaeiaaaaaaeiaaadaaceaaaaaaeiaaacaaaaaa
aaababaaabacacaaaaaaabaaacaaaaaaaaaaaaaaaaaaahaaadaaacaaaaaaaaaa
aaacppppfbaaaaafafaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaiadpfbaaaaaf
agaaapkaaaaaeaecaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaapla
bpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaacaaaaaaia
adaaadlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaac
aaaaaajaacaiapkaaiaaaaadaaaaciiaacaaoelaacaaoelaahaaaaacaaaacbia
aaaappiaabaaaaacabaaahiaacaaoelaaeaaaaaeabaachiaabaaoeiaaaaaaaia
abaaoelaafaaaaadaaaachiaaaaaaaiaacaaoelaceaaaaacacaachiaabaaoeia
abaaaaacabaaabiaaaaakklaabaaaaacabaaaciaaaaapplaecaaaaadabaacpia
abaaoeiaacaioekaecaaaaadadaaapiaaaaaoelaabaioekaecaaaaadaeaacpia
adaaoelaaaaioekaaeaaaaaeaeaacbiaabaappiaafaaaakaafaaffkaaeaaaaae
aeaacciaabaaffiaafaaaakaafaaffkafkaaaaaeaaaadiiaaeaaoeiaaeaaoeia
afaakkkaacaaaaadaaaaciiaaaaappibafaappkaahaaaaacaaaaciiaaaaappia
agaaaaacaeaaceiaaaaappiaaiaaaaadaaaaciiaaeaaoeiaacaaoeiaalaaaaad
abaaabiaaaaappiaafaakkkacaaaaaadaaaaaiiaabaaaaiaagaaaakaafaaaaad
abaaahiaaaaappiaaaaaoekaafaaaaadabaaahiaadaappiaabaaoeiaafaaaaad
acaachiaadaaoeiaaaaaoekaafaaaaadabaaahiaabaaoeiaacaaaakaafaaaaad
abaaahiaabaaoeiaabaaoekaaiaaaaadaaaaciiaaeaaoeiaabaaoelaalaaaaad
abaaciiaaaaappiaafaakkkaaeaaaaaeabaaahiaacaaoeiaabaappiaabaaoeia
acaaaaadaaaaaiiaaeaappiaaeaappiaafaaaaadabaachiaaaaappiaabaaoeia
aiaaaaadabaaciiaaaaaoeiaaeaaoeiaacaaaaadabaaciiaabaappibafaappka
ceaaaaacaaaachiaabaaoelaaiaaaaadaaaacbiaaaaaoeiaaeaaoeiaacaaaaad
abaaciiaaaaaaaiaabaappiacaaaaaadaaaaabiaabaappiaaeaaaakaafaaaaad
aaaaahiaaaaaaaiaadaaoekaaeaaaaaeaaaachiaabaaoeiaaaaaoeiaabaaoeia
abaaaaacaaaaciiaafaakkkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefc
jaafaaaaeaaaaaaageabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaafkaaaaad
aagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaaddcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadcaaaaajocaabaaaaaaaaaaaagbjbaaaadaaaaaaagaabaaaaaaaaaaa
agbjbaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegbcbaaa
adaaaaaabaaaaaahbcaabaaaaaaaaaaajgahbaaaaaaaaaaajgahbaaaaaaaaaaa
eeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaa
agaabaaaaaaaaaaajgahbaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaa
abaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaadcaaaaapdcaabaaaacaaaaaa
hgapbaaaacaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaa
aaaaialpaaaaialpaaaaaaaaaaaaaaaaapaaaaahicaabaaaaaaaaaaaegaabaaa
acaaaaaaegaabaaaacaaaaaaddaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaa
abeaaaaaaaaaiadpaaaaaaaiicaabaaaaaaaaaaadkaabaiaebaaaaaaaaaaaaaa
abeaaaaaaaaaiadpelaaaaafecaabaaaacaaaaaadkaabaaaaaaaaaaabaaaaaah
bcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaaaaaaaaadeaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaa
aaaaeaecbjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaihcaabaaa
aaaaaaaaagaabaaaaaaaaaaaegiccaaaaaaaaaaaabaaaaaaefaaaaajpcaabaaa
adaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaadaaaaaadiaaaaaihcaabaaa
adaaaaaaegacbaaaadaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaaagiacaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaacaaaaaabaaaaaahicaabaaa
aaaaaaaaegacbaaaacaaaaaaegbcbaaaacaaaaaadeaaaaahicaabaaaaaaaaaaa
dkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaa
adaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaaadaaaaaa
egbabaaaaeaaaaaaeghobaaaacaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaa
aaaaaaaadkaabaaaadaaaaaadkaabaaaadaaaaaadiaaaaahhcaabaaaaaaaaaaa
pgapbaaaaaaaaaaaegacbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaacaaaaaaaaaaaaaiicaabaaaaaaaaaaadkaabaiaebaaaaaa
aaaaaaaaabeaaaaaaaaaiadpbaaaaaahbcaabaaaabaaaaaaegbcbaaaacaaaaaa
egbcbaaaacaaaaaaeeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaah
hcaabaaaabaaaaaaagaabaaaabaaaaaaegbcbaaaacaaaaaabaaaaaahbcaabaaa
abaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaahicaabaaaaaaaaaaa
dkaabaaaaaaaaaaaakaabaaaabaaaaaacpaaaaaficaabaaaaaaaaaaadkaabaaa
aaaaaaaadiaaaaaiicaabaaaaaaaaaaadkaabaaaaaaaaaaaakiacaaaaaaaaaaa
ajaaaaaabjaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaaihcaabaaa
abaaaaaapgapbaaaaaaaaaaaegiccaaaaaaaaaaaaiaaaaaadcaaaaajhccabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaadgaaaaaf
iccabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahahaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadadaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklkl"
}

SubProgram "gles3 " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES3"
}

}
	}

#LINE 53

	} 
	Fallback "Diffuse"
  }