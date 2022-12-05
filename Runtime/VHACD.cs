using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.Collections;
using UnityEngine;
#if ODIN_INSPECTOR
using Sirenix.Utilities;
using Sirenix.OdinInspector;
#endif
using Rhinox.GUIUtils.Attributes;
using Rhinox.GUIUtils;
using Rhinox.Lightspeed;
using Sirenix.OdinInspector;
using UnityEditor;

#if UNITY_EDITOR && ODIN_INSPECTOR
using Sirenix.Utilities.Editor;
using UnityEditor;
#endif

namespace MeshProcess
{
    public class VHACD : MonoBehaviour
    {
        [ShowIf("@_generatedColliders != null && _generatedColliders.Count > 0"), ListDrawerSettings(IsReadOnly = true)]
        [InlineButton(nameof(ClearColliders), "Clear")]
        [SerializeField]
        private List<MeshCollider> _generatedColliders;
        
        public IList<MeshCollider> GeneratedColliders => _generatedColliders ?? (IList<MeshCollider>)Array.Empty<MeshCollider>();

        [Title("Parameters"), InlineProperty, HideLabel]
        [System.Serializable]
        public unsafe struct Parameters
        {
            public void Init()
            {
                m_resolution = 100000;
                m_concavity = 0.001;
                m_planeDownsampling = 4;
                m_convexhullDownsampling = 4;
                m_alpha = 0.05;
                m_beta = 0.05;
                m_pca = 0;
                m_mode = 0; // 0: voxel-based (recommended), 1: tetrahedron-based
                m_maxNumVerticesPerCH = 64;
                m_minVolumePerCH = 0.0001;
                m_callback = null;
                m_logger = null;
                m_convexhullApproximation = 1;
                m_oclAcceleration = 0;
                m_maxConvexHulls = 1024;
                m_projectHullVertices =
                    true; // This will project the output convex hull vertices onto the original source mesh to increase the floating point accuracy of the results
            }

            [Tooltip("maximum concavity")] [Range(0, 1)]
            public double m_concavity;

            [Tooltip("controls the bias toward clipping along symmetry planes")] [Range(0, 1)]
            public double m_alpha;

            [Tooltip("controls the bias toward clipping along revolution axes")] [Range(0, 1)]
            public double m_beta;

            [Tooltip("controls the adaptive sampling of the generated convex-hulls")] [Range(0, 0.01f)]
            public double m_minVolumePerCH;

            public void* m_callback;
            public void* m_logger;

            [Tooltip("maximum number of voxels generated during the voxelization stage")] [Range(10000, 64000000)]
            public uint m_resolution;

            [Tooltip("controls the maximum number of triangles per convex-hull")] [Range(4, 1024)]
            public uint m_maxNumVerticesPerCH;

            [Tooltip("controls the granularity of the search for the \"best\" clipping plane")] [Range(1, 16)]
            public uint m_planeDownsampling;

            [Tooltip("controls the precision of the convex-hull generation process during the clipping plane selection stage")]
            [Range(1, 16)]
            public uint m_convexhullDownsampling;

            [Tooltip("enable/disable normalizing the mesh before applying the convex decomposition")] [Range(0, 1)]
            public uint m_pca;

            [Tooltip("0: voxel-based (recommended), 1: tetrahedron-based")] [Range(0, 1)]
            public uint m_mode;

            [Range(0, 1)] public uint m_convexhullApproximation;

            [Range(0, 1)] public uint m_oclAcceleration;

            public uint m_maxConvexHulls;

            [Tooltip(
                "This will project the output convex hull vertices onto the original source mesh to increase the floating point accuracy of the results")]
            public bool m_projectHullVertices;
        };

        unsafe struct ConvexHull
        {
            public double* m_points;
            public uint* m_triangles;
            public uint m_nPoints;
            public uint m_nTriangles;
            public double m_volume;
            public fixed double m_center[3];
        };

        [DllImport("libvhacd")]
        static extern unsafe void* CreateVHACD();

        [DllImport("libvhacd")]
        static extern unsafe void DestroyVHACD(void* pVHACD);

        [DllImport("libvhacd")]
        static extern unsafe bool ComputeFloat(
            void* pVHACD,
            float* points,
            uint countPoints,
            uint* triangles,
            uint countTriangles,
            Parameters* parameters);

        [DllImport("libvhacd")]
        static extern unsafe bool ComputeDouble(
            void* pVHACD,
            double* points,
            uint countPoints,
            uint* triangles,
            uint countTriangles,
            Parameters* parameters);

        [DllImport("libvhacd")]
        static extern unsafe uint GetNConvexHulls(void* pVHACD);

        [DllImport("libvhacd")]
        static extern unsafe void GetConvexHull(
            void* pVHACD,
            uint index,
            ConvexHull* ch);

        public Parameters m_parameters;

        public VHACD()
        {
            m_parameters.Init();
        }

        private unsafe void GenerateVHACD(Mesh mesh, out void* vhacd, out uint numHulls)
        {
            vhacd = CreateVHACD();
            var parameters = m_parameters;

            var verts = mesh.vertices;
            var tris = mesh.triangles;
            fixed (Vector3* pVerts = verts)
            fixed (int* pTris = tris)
            {
                ComputeFloat(
                    vhacd,
                    (float*) pVerts, (uint) verts.Length,
                    (uint*) pTris, (uint) tris.Length / 3,
                    &parameters);
            }

            numHulls = GetNConvexHulls(vhacd);
        }

        [ContextMenu("Clear Generated Colliders")]
        private void ClearColliders()
        {
            if (!_generatedColliders.IsNullOrEmpty())
            {
                foreach (var collider in _generatedColliders)
                    DestroyImmediate(collider);
                _generatedColliders.Clear();
            }

            if (_generatedColliders == null)
                _generatedColliders = new List<MeshCollider>();
        }

        public unsafe List<Mesh> GenerateConvexMeshes(void* vhacd, int numHulls)
        {
            List<Mesh> convexMesh = new List<Mesh>(numHulls);
            for (uint index = 0; index < numHulls; ++index)
            {
                ConvexHull hull;
                GetConvexHull(vhacd, index, &hull);

                var hullMesh = new Mesh();
                var hullVerts = new Vector3[hull.m_nPoints];
                fixed (Vector3* pHullVerts = hullVerts)
                {
                    var pComponents = hull.m_points;
                    var pVerts = pHullVerts;

                    for (var pointCount = hull.m_nPoints; pointCount != 0; --pointCount)
                    {
                        pVerts->x = (float) pComponents[0];
                        pVerts->y = (float) pComponents[1];
                        pVerts->z = (float) pComponents[2];

                        pVerts += 1;
                        pComponents += 3;
                    }
                }

                hullMesh.SetVertices(hullVerts);

                var indices = new int[hull.m_nTriangles * 3];
                Marshal.Copy((System.IntPtr) hull.m_triangles, indices, 0, indices.Length);
                hullMesh.SetTriangles(indices, 0);


                convexMesh.Add(hullMesh);
            }

            return convexMesh;
        }

        public unsafe void GenerateVHACDScript()
        {
            var meshFilters = GetComponentsInChildren<MeshFilter>(true);
            if (!meshFilters.Any())
            {
                Debug.LogWarning("You need a meshfilter(s) to generate a v-vacd collider.");
                return;
            }

            List<VHACD_Info> infos = new List<VHACD_Info>();
            foreach (var meshFilter in meshFilters)
            {
                var mesh = meshFilter.sharedMesh;
                if (mesh && mesh.isReadable)
                {
                    if (mesh.bounds.size.x > 0.0f && mesh.bounds.size.y > 0.0f && mesh.bounds.size.z > 0.0f)
                    {                        
                        GenerateVHACD(meshFilter.sharedMesh, out var vhacd, out var numHulls);
                        infos.Add(new VHACD_Info
                        {
                            MeshFilter = meshFilter,
                            NumHulls = (int) numHulls,
                            VHACD = vhacd
                        });
                    }
                    else
                    {
                        Debug.LogWarning("Can't calculate VHACD on mesh with a dimensions of size 0");
                    }
                }
            }

            ClearColliders();

            foreach (var info in infos)
            {
                var root = info.GetOrCreateRoot();
                var meshes = GenerateConvexMeshes(info.VHACD, info.NumHulls);

                foreach (var mesh in meshes)
                {
                    var collider = root.gameObject.AddComponent<MeshCollider>();
                    collider.sharedMesh = mesh;
                    collider.convex = true;

                    _generatedColliders.Add(collider);
                }
            }
        }


        private const string GEN_NAME = "[Generated] VHACD";
        private unsafe struct VHACD_Info
        {
            public void* VHACD;
            public int NumHulls;
            public MeshFilter MeshFilter;

            public Transform GetOrCreateRoot()
            {
                var root = MeshFilter.transform.Find(GEN_NAME);
                if (root == null)
                {
                    root = new GameObject(GEN_NAME).transform;
                    root.SetParent(MeshFilter.transform, false);
                }

                return root;
            }

        }


#if UNITY_EDITOR
        [ContextMenu("Generate VHACD")]
        private unsafe void GenerateVHACDCollision()
        {
            var meshFilters = GetComponentsInChildren<MeshFilter>(true);
            if (!meshFilters.Any())
            {
                Debug.LogWarning("You need a meshfilter(s) to generate a v-vacd collider.");
                return;
            }

            List<VHACD_Info> infos = new List<VHACD_Info>();
            foreach (var meshFilter in meshFilters)
            {
                var mesh = meshFilter.sharedMesh;
                if (mesh)
                {
                    if (mesh.bounds.size.x > 0.0f && mesh.bounds.size.y > 0.0f && mesh.bounds.size.z > 0.0f)
                    {                        
                        GenerateVHACD(meshFilter.sharedMesh, out var vhacd, out var numHulls);
                        infos.Add(new VHACD_Info
                        {
                            MeshFilter = meshFilter,
                            NumHulls = (int) numHulls,
                            VHACD = vhacd
                        });
                    }
                    else
                    {
                        Debug.LogWarning("Can't calculate VHACD on mesh with a dimensions of size 0");
                    }
                }
            }

            var totalMeshes = infos.Sum(x => x.NumHulls);

            if (totalMeshes > 10)
            {
                var cont = EditorUtility.DisplayDialog(
                    "Lots of meshes",
                    $"A total of {totalMeshes} colliders will be generated. Are you sure you wish to continue?",
                    "Yes", "No");

                if (!cont) return;
            }

            ClearColliders();

            foreach (var info in infos)
            {
                var root = info.GetOrCreateRoot();
                var meshes = GenerateConvexMeshes(info.VHACD, info.NumHulls);

                foreach (var mesh in meshes)
                {
                    var collider = root.gameObject.AddComponent<MeshCollider>();
                    collider.sharedMesh = mesh;
                    collider.convex = true;

                    _generatedColliders.Add(collider);
                }
            }
        }
#endif
    }
}
