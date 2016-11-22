using UnityEngine;
using System.Collections;

public class ShapeMatching : MonoBehaviour {

    [Range(0,0.1f)]
    public float hard = 0.02f;
    [Range(0, 1.0f)]
    public float damp =0.94f;
    public Vector3 force = Vector3.zero;

    private static ShapeMatching instance;
    public static ShapeMatching Instance
    {
        get
        {
            if (instance == null)
            {
                Debug.LogError("ShapeMatching instance does not exist");
            }
            return instance;
        }
    }
    void Awake()
    {
        instance = this;
        p = Matrix4x4.identity;
    }
    Matrix4x4 p;
    public void ShapeMatchingDynamics(ref Vector3[] vertices,ref Vector3[] velocity)
    {
        //calculate the center 
        Vector3 c0, c;
        Vector3 sum0=Vector3.zero, sum = Vector3.zero;
        for(int i = 0; i < vertices.Length; i++)
        {
            sum0 += CubeMesh.vertices[i];
            sum += vertices[i];
        }
        c0 = sum0 / vertices.Length;
        c = sum / vertices.Length;

        //calculate the moment matrix
        Matrix4x4 apq = Matrix4x4.zero;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector4 qi = CubeMesh.vertices[i] - c0; 
            Vector4 pi = vertices[i] - c;
            Matrix4x4 qm = new Matrix4x4();
            Matrix4x4 pm = new Matrix4x4();
            pm.SetColumn(0, pi);
            qm.SetRow(0, qi);
            apq = MatrixAdd(apq, pm * qm);
        }

        //calculate the matrix P and D
        Matrix4x4 d = apq.transpose * apq;
        Jacobi(ref p, ref d, 12, 0.01f);
        //calculate the matrix S
        Matrix4x4 s = p * MatrixSqrt(d) * p.transpose;
        //calculate the matrix R
        Matrix4x4 r = apq * s.inverse;
        //calculate the goal position
        Vector3[] goalPosition = new Vector3[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 qi = CubeMesh.vertices[i] - c0;
            goalPosition[i] = r.MultiplyVector(qi) + c;
        }
        //update the vertex position and speed
        for (int i = 0; i < vertices.Length; i++)
        {
            velocity[i] += hard / Time.deltaTime * (goalPosition[i] - vertices[i]) + Time.deltaTime * force;
            vertices[i] += Time.deltaTime * velocity[i];
            velocity[i] *= damp;
        }
    }
    void Jacobi(ref Matrix4x4 p, ref Matrix4x4 m,int maxNum,float thresh)
    {
        if (!p.isIdentity)
        {
            m = p.transpose * m * p;
        }

        for(int i = 0; i < maxNum; i++)
        {
            Matrix4x4 r = Matrix4x4.identity; 

            if (m.m01 >= m.m02 && m.m01 >= m.m12)
            {
                if (Mathf.Abs(m.m01) < thresh) return;
                float theta = Mathf.Atan2(-2 * m.m01, m.m00 - m.m11)/2;
                r.m00 = Mathf.Cos(theta);
                r.m01 = Mathf.Sin(theta);
                r.m10 = -Mathf.Sin(theta);
                r.m11 = Mathf.Cos(theta);
            }
            else if (m.m02 >= m.m01 && m.m02 >= m.m12)
            {
                if (Mathf.Abs(m.m02) < thresh) return;
                float theta = Mathf.Atan2(-2 * m.m02, m.m00 - m.m22)/2;
                r.m00 = Mathf.Cos(theta);
                r.m02 = Mathf.Sin(theta);
                r.m20 = -Mathf.Sin(theta);
                r.m22 = Mathf.Cos(theta);
            }
            else if (m.m12 >= m.m01 && m.m12 >= m.m02)
            {
                if (Mathf.Abs(m.m12) < thresh) return;
                float theta = Mathf.Atan2(-2 * m.m12, m.m11 - m.m22)/2;
                r.m11 = Mathf.Cos(theta);
                r.m12 = Mathf.Sin(theta);
                r.m21 = -Mathf.Sin(theta);
                r.m22 = Mathf.Cos(theta);
            }

            m = r.transpose * (m * r);
            p = p * r;
        }
    }
    Matrix4x4 MatrixAdd(Matrix4x4 a,Matrix4x4 b)
    {
        Matrix4x4 c = Matrix4x4.zero;
        c.SetColumn(0, a.GetColumn(0) + b.GetColumn(0));
        c.SetColumn(1, a.GetColumn(1) + b.GetColumn(1));
        c.SetColumn(2, a.GetColumn(2) + b.GetColumn(2));
        return c;
    }
    Matrix4x4 MatrixSqrt(Matrix4x4 m)
    {
        Matrix4x4 t=Matrix4x4.zero;
        t.m00 = Mathf.Sqrt(m.m00);
        t.m11 = Mathf.Sqrt(m.m11);
        t.m22 = Mathf.Sqrt(m.m22);
        t.m33 = 1;
        return t;
    }
}
