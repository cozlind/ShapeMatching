using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class MeshController : MonoBehaviour
{
    MeshCollider meshCollider;
    MeshFilter meshFilter;
    Mesh mesh;
    Vector3[] vertices;
    Vector3[] velocity;
    void Start()
    {
        meshCollider = GetComponent<MeshCollider>();
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;

        GameObject temp = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Mesh cubeMesh = temp.GetComponent<MeshFilter>().mesh;
        CubeMesh.vertices = cubeMesh.vertices;
        CubeMesh.uvs = cubeMesh.uv;
        CubeMesh.indices = cubeMesh.GetIndices(0);
        CubeMesh.triangles = cubeMesh.triangles;
        Destroy(temp);

        vertices = CubeMesh.InitialVertices;
        velocity = new Vector3[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            velocity[i] = Vector3.zero;
        }
        lastMousePosition = Vector3.zero;
    }
    Vector3 lastMousePosition;
    void Update()
    {
        UpdateMesh(vertices,velocity);
        RaycastHit hit;
        Vector3 choosePoint=Vector3.zero;
        if (Input.GetMouseButtonDown(0)&&Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition),out hit)){
            choosePoint=transform.InverseTransformPoint(hit.point);
            float dst = 0.5f;
            for (int i = 0; i < vertices.Length; i++)
            {
                if (Vector3.Distance(vertices[i],choosePoint)<= dst)
                {
                    dst = Vector3.Distance(vertices[i], choosePoint);
                    velocity[i] += hit.normal * 13;
                }
            }
        }
    }
    void UpdateMesh(Vector3[] vertices,Vector3[] velocity)
    {
        ShapeMatching.Instance.ShapeMatchingDynamics(ref vertices,ref velocity);
        mesh.vertices = vertices;
        mesh.uv = CubeMesh.uvs;
        mesh.triangles = CubeMesh.triangles;
        mesh.RecalculateNormals();
        meshCollider.sharedMesh = mesh;
    }
}
struct CubeMesh
{
    public static Vector2[] uvs;
    public static int[] indices;
    public static int[] triangles;
    public static Vector3[] vertices;
    public static Vector3[] InitialVertices = new Vector3[] {
        new Vector3(8f, -8f, 8f),
        new Vector3(-8f, -8f, 8f),
        new Vector3(5f, 5f, 5f),
        new Vector3(-5f, 5f, 5f),

        new Vector3(7f, 7f, -7f),
        new Vector3(-5f, 5f, -5f),
        new Vector3(5f, -5f, -5f),
        new Vector3(-6f, -6f, -6f),

        new Vector3(5f, 5f, 5f),
        new Vector3(-5f, 5f, 5f),
        new Vector3(7f, 7f, -7f),
        new Vector3(-5f, 5f, -5f),

        new Vector3(5f, -5f, -5f),
        new Vector3(8f, -8f, 8f),
        new Vector3(-8f, -8f, 8f),
        new Vector3(-6f, -6f, -6f),

        new Vector3(-8f, -8f, 8f),
        new Vector3(-5f, 5f, 5f),
        new Vector3(-5f, 5f, -5f),
        new Vector3(-6f, -6f, -6f),

        new Vector3(5f, -5f, -5f),
        new Vector3(7f, 7f, -7f),
        new Vector3(5f, 5f, 5f),
        new Vector3(8f, -8f, 8f),
    };
}