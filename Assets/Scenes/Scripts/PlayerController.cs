using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class PlayerController : MonoBehaviour
{
    [Header("Movement")]
    public float moveSpeed = 5f;
    public float sprintMultiplier = 1.5f;

    [Header("Jump / Gravity")]
    public float jumpHeight = 1.5f;
    public float gravity = -9.81f;

    private CharacterController controller;
    private float verticalVelocity = 0f;

    void Awake()
    {
        controller = GetComponent<CharacterController>();
    }

    void Update()
    {
        HandleMovement();
    }

    private void HandleMovement()
    {
        // Read input
        float inputX = Input.GetAxisRaw("Horizontal"); // A/D
        float inputZ = Input.GetAxisRaw("Vertical");   // W/S

        // Camera-relative movement if camera exists, otherwise local
        Vector3 forward = Camera.main ? Camera.main.transform.forward : transform.forward;
        Vector3 right = Camera.main ? Camera.main.transform.right : transform.right;
        forward.y = 0f;
        right.y = 0f;
        forward.Normalize();
        right.Normalize();

        Vector3 moveDirection = (right * inputX + forward * inputZ).normalized;

        // Speed / sprint
        float speed = moveSpeed;
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
            speed *= sprintMultiplier;

        Vector3 horizontalVelocity = moveDirection * speed;

        // Ground check and jump
        if (controller.isGrounded)
        {
            // Reset downward velocity when grounded
            if (verticalVelocity < 0f)
                verticalVelocity = -1f;

            if (Input.GetKeyDown(KeyCode.Space))
            {
                // v = sqrt(2 * g * h) but gravity is negative
                verticalVelocity = Mathf.Sqrt(jumpHeight * -2f * gravity);
            }
        }
        else
        {
            // apply gravity while in air
            verticalVelocity += gravity * Time.deltaTime;
        }

        Vector3 velocity = horizontalVelocity + Vector3.up * verticalVelocity;

        controller.Move(velocity * Time.deltaTime);
    }
}
