using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;
    public Vector3 offset = new Vector3(0f, 3.5f, -7f);
    public float positionSmooth = 5f;
    public float rotationSmooth = 6f;

    private Vector3 velocity;

    void LateUpdate()
    {
        Vector3 desiredPos = target.position +
                             target.forward * offset.z +
                             target.up * offset.y;

        transform.position = Vector3.SmoothDamp(
            transform.position,
            desiredPos,
            ref velocity,
            1f / positionSmooth
        );

        Quaternion desiredRot = Quaternion.LookRotation(
            target.forward,
            Vector3.up
        );

        transform.rotation = Quaternion.Slerp(
            transform.rotation,
            desiredRot,
            rotationSmooth * Time.deltaTime
        );
    }
}
