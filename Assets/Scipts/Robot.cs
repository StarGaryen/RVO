using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Robot : MonoBehaviour
{

    public Vector2 VMax;
    [HideInInspector]
    public Vector2 DesriedVelocity;
    public Transform target;

    private Rigidbody2D myRigidBody;
    private bool reached = false;


    private Transform sprite;
    // Use this for initialization
    void Start()
    {
        myRigidBody= GetComponent<Rigidbody2D>();
        sprite = transform.GetChild(1);
    }

    // Update is called once per frame
    void Update()
    {
        
        
         
    }

    public Vector2 computeDesiredVelocity()
    {
        if (!reached)
        {
            Vector3 diff3 = target.position - transform.position;
            DesriedVelocity = diff3;
            DesriedVelocity.Normalize();
            DesriedVelocity.Scale(VMax);
        }
        else
        {
            DesriedVelocity = Vector2.zero;
        }
        //Debug.Log("desired velocity :: " + DesriedVelocity);
        return DesriedVelocity;

    }
    void OnCollisionEnter2D(Collision2D other)
    {
        if (other.gameObject.Equals(target.gameObject))
        {
            reached = true;
            myRigidBody.velocity = Vector2.zero;
            Debug.Log(gameObject.name + " reached target");
            this.enabled = false;
        }
        else
        {
            //Debug.Log(gameObject.name + " hit " + other.gameObject.name);
        }
    }
    

    public Vector2 getVelocity()
    {
        return myRigidBody.velocity;
    }
    public void setVelocity(Vector2 vel)
    {
        myRigidBody.velocity = vel;
        Debug.Log(gameObject.name+ " new velocity = " + vel);
        sprite.rotation = Quaternion.Euler(0f, 0f, Mathf.Rad2Deg * Mathf.Atan2(myRigidBody.velocity.y, myRigidBody.velocity.x));
        Vector3 scale;
        scale.x = scale.y = scale.z = 0.3f * (myRigidBody.velocity.magnitude / VMax.magnitude);
        sprite.localScale = scale;
    }
}
