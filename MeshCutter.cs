using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class MeshCutter : MonoBehaviour
{
    public Vector3 cutPlanePoint;
    public Vector3 cutPlaneNormal;

    [Space] public bool drawGizmo = true;
    private Mesh mesh;

    HashSet<int> vertexIndexToMove = new HashSet<int>();
    HashSet<int> vertexIndexNotToMove = new HashSet<int>();
    private Vector3[] vertices;
    private GameObject anchor;

    void Start()
    {
        mesh = GetComponent<MeshFilter>().mesh;
        var planeNormalLocal = transform.InverseTransformDirection(cutPlaneNormal);
        var planePointLocal = transform.InverseTransformPoint(cutPlanePoint);
        Vector4 plane = new Vector4(planeNormalLocal.x, planeNormalLocal.y, planeNormalLocal.z, -Vector3.Dot(planeNormalLocal, planePointLocal));
        CutMesh(mesh, plane);
        anchor = new GameObject();
        vertices = mesh.vertices;
        var triangles = mesh.triangles;
        var extraPoints = new List<int>();
        print(vertexIndexToMove.Count);
        for (var i = 0; i < triangles.Length; i += 3)
        {
            bool match = true;
            for (int j = 0; j < 3; j++)
            {
                if (Vector4.Dot(new Vector4(vertices[triangles[i + j]].x, vertices[triangles[i + j]].y, vertices[triangles[i + j]].z, 1), plane) < 0)

                {
                    match = false;
                }
            }

            if (vertexIndexNotToMove.Contains(triangles[i]) || vertexIndexNotToMove.Contains(triangles[i + 1]) || vertexIndexNotToMove.Contains(triangles[i + 2]))
                match = false;
            if (match)
            {
                extraPoints.Add(triangles[i]);
                extraPoints.Add(triangles[i + 1]);
                extraPoints.Add(triangles[i + 2]);
            }
        }

        extraPoints.ForEach(x => vertexIndexToMove.Add(x));
        prevAnchorPosition = anchor.transform.position;
    }

    void CutMesh(Mesh mesh, Vector4 plane)
    {
        List<List<int>> commonTriangles = new List<List<int>>(); // new List<int>(mesh.triangles);
        List<Vector3> vertices = new List<Vector3>(mesh.vertices);
        List<Vector3> normals = new List<Vector3>(mesh.normals);
        List<Vector2> uvs = new List<Vector2>(mesh.uv);
        List<Vector4> tangents = new List<Vector4>(mesh.tangents);
        int verticesCount = vertices.Count;

        for (int smIndex = 0; smIndex < mesh.subMeshCount; smIndex++)
        {
            List<int> triangles = new List<int>();
            mesh.GetTriangles(triangles, smIndex);
            int trianglesCount = triangles.Count;
            for (var i = 0; i < trianglesCount; i += 3)
            {
                int numCross = 0;
                Vector3 p0 = vertices[triangles[i]];
                Vector3 p1 = vertices[triangles[i + 1]];
                Vector3 p2 = vertices[triangles[i + 2]];
                List<Vector3?> crosses = new List<Vector3?>
                    {
                        PlaneCrossSegment(p0, p1, plane),
                        PlaneCrossSegment(p1, p2, plane),
                        PlaneCrossSegment(p2, p0, plane),
                    }
                    ; // точки пересечения 
                numCross += crosses.FindAll(x => x != null).Count;

                if (numCross == 1) // TODO---------------
                {
                    int index = 0;
                    for (var j = 0; j < crosses.Count; j++)
                    {
                        if (crosses[j] != null)
                            index = j;
                    }

                    int v1 = index % 3;
                    int v2 = (index + 1) % 3;
                    int v3 = (index + 2) % 3;
                    //continue;
                }

                if (numCross == 2)
                {
                    int index0 = -1;
                    int index1 = -1;

                    for (var j = 0; j < crosses.Count; j++)
                    {
                        if (crosses[j] != null)
                        {
                            if (index0 == -1)
                            {
                                index0 = j;
                                continue;
                            }

                            if (index1 == -1)
                            {
                                index1 = j;
                            }
                        }
                    }

                    int v0 = -1;
                    int v1 = -1;
                    int v2 = -1;

                    if (index0 == 0 && index1 == 1)
                    {
                        v0 = 0;
                        v1 = 1;
                        v2 = 2;
                    }

                    if (index0 == 1 && index1 == 2)
                    {
                        v0 = 1;
                        v1 = 2;
                        v2 = 0;
                    }

                    if (index0 == 0 && index1 == 2)
                    {
                        v0 = 2;
                        v1 = 0;
                        v2 = 1;
                        int t = index0;
                        index0 = index1;
                        index1 = t;
                    }

                    Vector2 index0_uv = uvs[triangles[i + v0]] + (uvs[triangles[i + v1]] - uvs[triangles[i + v0]]) *
                                        (Vector3.Distance(vertices[triangles[i + v0]], (Vector3) crosses[index0]) /
                                         Vector3.Distance(vertices[triangles[i + v0]], vertices[triangles[i + v1]]));
                    Vector2 index1_uv = uvs[triangles[i + v1]] + (uvs[triangles[i + v2]] - uvs[triangles[i + v1]]) *
                                        (Vector3.Distance(vertices[triangles[i + v1]], (Vector3) crosses[index1]) /
                                         Vector3.Distance(vertices[triangles[i + v1]], vertices[triangles[i + v2]]));

                    Vector3 index0_normal = normals[triangles[i + v0]] + (normals[triangles[i + v1]] - normals[triangles[i + v0]]) *
                                            (Vector3.Distance(vertices[triangles[i + v0]], (Vector3) crosses[index0]) /
                                             Vector3.Distance(vertices[triangles[i + v0]], vertices[triangles[i + v1]]));

                    Vector3 index1_normal = normals[triangles[i + v1]] + (normals[triangles[i + v2]] - normals[triangles[i + v1]]) *
                                            (Vector3.Distance(vertices[triangles[i + v1]], (Vector3) crosses[index1]) /
                                             Vector3.Distance(vertices[triangles[i + v1]], vertices[triangles[i + v2]]));

                    Vector3 index0_tangents = tangents[triangles[i + v0]] + (tangents[triangles[i + v1]] - tangents[triangles[i + v0]]) *
                                              (Vector3.Distance(vertices[triangles[i + v0]], (Vector3) crosses[index0]) /
                                               Vector3.Distance(vertices[triangles[i + v0]], vertices[triangles[i + v1]]));

                    Vector3 index1_tangents = tangents[triangles[i + v1]] + (tangents[triangles[i + v2]] - tangents[triangles[i + v1]]) *
                                              (Vector3.Distance(vertices[triangles[i + v1]], (Vector3) crosses[index1]) /
                                               Vector3.Distance(vertices[triangles[i + v1]], vertices[triangles[i + v2]]));
                    vertices.Add(vertices[triangles[i + v0]]);
                    uvs.Add(uvs[triangles[i + v0]]);
                    normals.Add(normals[triangles[i + v0]]);
                    tangents.Add(tangents[triangles[i + v0]]);
                    var V0 = new Point(vertices.Last(), vertices.Count - 1);

                    vertices.Add(vertices[triangles[i + v1]]);
                    uvs.Add(uvs[triangles[i + v1]]);
                    normals.Add(normals[triangles[i + v1]]);
                    tangents.Add(tangents[triangles[i + v1]]);
                    var V1 = new Point(vertices.Last(), vertices.Count - 1);

                    vertices.Add(vertices[triangles[i + v2]]);
                    uvs.Add(uvs[triangles[i + v2]]);
                    normals.Add(normals[triangles[i + v2]]);
                    tangents.Add(tangents[triangles[i + v2]]);
                    var V2 = new Point(vertices.Last(), vertices.Count - 1);

                    vertices.Add((Vector3) crosses[index0]);
                    uvs.Add(index0_uv);
                    normals.Add(index0_normal);
                    tangents.Add(index0_tangents);
                    var Index0 = new Point(vertices.Last(), vertices.Count - 1);

                    vertices.Add((Vector3) crosses[index0]);
                    uvs.Add(index0_uv);
                    normals.Add(index0_normal);
                    tangents.Add(index0_tangents);
                    var Index0_copy = new Point(vertices.Last(), vertices.Count - 1);

                    vertices.Add((Vector3) crosses[index1]);
                    uvs.Add(index1_uv);
                    normals.Add(index1_normal);
                    tangents.Add(index1_tangents);
                    var Index1 = new Point(vertices.Last(), vertices.Count - 1);

                    vertices.Add((Vector3) crosses[index1]);
                    uvs.Add(index1_uv);
                    normals.Add(index1_normal);
                    tangents.Add(index1_tangents);
                    var Index1_copy = new Point(vertices.Last(), vertices.Count - 1);
                    triangles[i] = V0.index;
                    triangles[i + 1] = Index0.index;
                    triangles[i + 2] = Index1.index;

                    triangles.AddRange(new List<int> {Index1.index, V2.index, V0.index});
                    triangles.AddRange(new List<int> {Index0.index, Index0_copy.index, Index1.index});
                    triangles.AddRange(new List<int> {Index0_copy.index, Index1_copy.index, Index1.index});
                    triangles.AddRange(new List<int> {Index0_copy.index, V1.index, Index1_copy.index});

                    if (Vector4.Dot(new Vector4(V0.point.x, V0.point.y, V0.point.z, 1), plane) > 0)
                    {
                        vertexIndexToMove.Add(Index0.index);
                        vertexIndexToMove.Add(Index1.index);
                        vertexIndexToMove.Add(V0.index);
                        vertexIndexToMove.Add(V2.index);

                        vertexIndexNotToMove.Add(Index0_copy.index);
                        vertexIndexNotToMove.Add(Index1_copy.index);
                        vertexIndexNotToMove.Add(V1.index);
                    }

                    if (Vector4.Dot(new Vector4(V1.point.x, V1.point.y, V1.point.z, 1), plane) > 0)
                    {
                        vertexIndexToMove.Add(Index0_copy.index);
                        vertexIndexToMove.Add(Index1_copy.index);
                        vertexIndexToMove.Add(V1.index);

                        vertexIndexNotToMove.Add(Index0.index);
                        vertexIndexNotToMove.Add(Index1.index);
                        vertexIndexNotToMove.Add(V0.index);
                        vertexIndexNotToMove.Add(V2.index);
                    }

                    //continue;
                }
            }

            //commonTriangles.AddRange(triangles);
            commonTriangles.Add(triangles);
            //mesh.SetTriangles(triangles, smIndex,false);
        }

        mesh.vertices = vertices.ToArray();
        for (var i = 0; i < commonTriangles.Count; i++)
        {
            mesh.SetTriangles(commonTriangles[i], i, false);
        }

        //mesh.SetTriangles(); = commonTriangles.ToArray();
        mesh.normals = normals.ToArray();
        mesh.tangents = tangents.ToArray();
        mesh.uv = uvs.ToArray();
        mesh.RecalculateBounds();
        //mesh.RecalculateNormals();
        //mesh.RecalculateTangents();
    }

    Vector3? PlaneCrossSegment(Vector3 p1, Vector3 p2, Vector4 plane)
    {
        float dot1 = Vector4.Dot(new Vector4(p1.x, p1.y, p1.z, 1), plane);
        float dot2 = Vector4.Dot(new Vector4(p2.x, p2.y, p2.z, 1), plane);
        if (dot1 < 0 && dot2 > 0 || dot1 > 0 && dot2 < 0)
        {
            Vector3 n = p2 - p1;
            float t = -Vector4.Dot(plane, new Vector4(p1.x, p1.y, p1.z, 1)) / (Vector4.Dot(plane, new Vector4(n.x, n.y, n.z, 0)));
            return new Vector3(n.x * t + p1.x, n.y * t + p1.y, n.z * t + p1.z);
        }
        else
            return null; // dot1 < 0 && dot2 > 0 || dot1 > 0 && dot2 < 0;
    }

    private Vector3 prevAnchorPosition;

    void Update()
    {
        foreach (var i in vertexIndexToMove)
        {
            vertices[i] += transform.InverseTransformPoint(anchor.transform.position) - transform.InverseTransformPoint(prevAnchorPosition);
        }

        prevAnchorPosition = anchor.transform.position;
        mesh.vertices = vertices;
        //mesh.RecalculateBounds();
        //mesh.RecalculateNormals();
        //mesh.RecalculateTangents();
    }

    private void OnDrawGizmos()
    {
        if (!drawGizmo) return;
        Gizmos.color = Color.red;
        Matrix4x4 origin = Gizmos.matrix;
        Matrix4x4 rotationMatrix = Matrix4x4.TRS(
            cutPlanePoint,
            Quaternion.LookRotation(Vector3.ProjectOnPlane(cutPlaneNormal + Vector3.forward, cutPlaneNormal), cutPlaneNormal),
            new Vector3(1, 0.01f, 1));
        Gizmos.matrix = rotationMatrix;

        Gizmos.DrawCube(Vector3.zero, Vector3.one);
        Gizmos.matrix = origin;

        Gizmos.DrawLine(cutPlanePoint, cutPlanePoint + cutPlaneNormal.normalized);
        //Gizmos.DrawLine();
    }
}

public class Point
{
    public Vector3 point;
    public int index;

    public Point(Vector3 point, int index)
    {
        this.point = point;
        this.index = index;
    }
}