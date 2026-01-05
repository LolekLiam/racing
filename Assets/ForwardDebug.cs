using UnityEngine;

public class ForwardDebug : MonoBehaviour
{
    void OnDrawGizmos()
    {
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position,
            transform.position + transform.forward * 3f);

        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position,
            transform.position + transform.right * 3f);

        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position,
            transform.position + transform.up * 3f);
    }
}
