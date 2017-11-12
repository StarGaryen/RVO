using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Manager : MonoBehaviour {

    public float RoboRadius = 0.2f;
    public float radiusBuffer = 0.01f;
    public Transform[] obstacles;
    public Robot[] robots;
    private float twiceRadius;
	// Use this for initialization
	void Start () {
        twiceRadius = 2 * (RoboRadius + radiusBuffer);
    }
	
	// Update is called once per frame
	void Update () {
		for(int i =0;i< robots.Length; i++)
        {
            robots[i].computeDesiredVelocity();
        }
        RVO_Update();
	}
    private void RVO_Update()
    {
        float safeRad = RoboRadius +radiusBuffer;

        for (int i = 0; i < robots.Length; i++)
        {
           //if (robots[i].isActiveAndEnabled)
          // {
                Vector2 VA = robots[i].getVelocity();
                Vector2 PA = robots[i].transform.position;
                ArrayList RVO_All = new ArrayList();
                for (int j = 0; j < robots.Length; j++)
                {
                    if (i != j)
                    {
                        Vector2 VB = robots[j].getVelocity();
                        Vector2 PB = robots[j].transform.position;
                        RVO_BA BA = new RVO_BA();
                        BA.avoidRadius = twiceRadius;
                        if (robots[j].isActiveAndEnabled)
                        {
                            BA.trans.Set(PA.x + 0.5f * (VA.x + VB.x), PA.y + 0.5f * (VA.y + VB.y));
                        }
                        else
                        {
                            // BA.trans.Set(PB.x - PA.x, PB.y - PA.y);
                            BA.trans.Set(PA.x + 0.5f * (VA.x + VB.x), PA.y + 0.5f * (VA.y + VB.y));
                        }
                        BA.dist = Vector2.Distance(PA, PB);
                        float theta_BA = Mathf.Atan2(PB.y - PA.y, PB.x - PA.y);
                        if (twiceRadius > BA.dist)
                        {
                            BA.dist = twiceRadius;
                        }
                        float thetaBAOrt = Mathf.Asin(twiceRadius / BA.dist);
                        float thetaOrtLeft = theta_BA + thetaBAOrt;
                        BA.leftBound.Set(Mathf.Cos(thetaOrtLeft), Mathf.Sin(thetaOrtLeft));
                        float thetaOrtRight = theta_BA - thetaBAOrt;
                        BA.rightBound.Set(Mathf.Cos(thetaOrtRight), Mathf.Sin(thetaOrtRight));
                        RVO_All.Add(BA);
                    }
                }
                for (int j = 0; j < obstacles.Length; j++)
                {
                    Vector2 VB = Vector2.zero;
                    Vector2 PB = obstacles[j].transform.position;
                    RVO_BA BA = new RVO_BA();
                    Debug.Log(obstacles[j].name + " location " + PB);
                    BA.trans.Set(PA.x + 0.5f * (VA.x + VB.x), PA.y + 0.5f * (VA.y + VB.y));
                    BA.dist = Vector2.Distance(PA, PB);
                    float theta_BA = Mathf.Atan2(PB.y - PA.y, PB.x - PA.x);
                    float rad =0.5f* 1.5f* obstacles[j].lossyScale.x;
                    BA.avoidRadius = rad + RoboRadius;
                    if (BA.avoidRadius > BA.dist)
                    {
                        BA.dist = BA.avoidRadius;
                    }
                    float thetaBAOrt = Mathf.Asin(BA.avoidRadius / BA.dist);
                    float thetaOrtLeft = theta_BA + thetaBAOrt;
                    BA.leftBound.Set(Mathf.Cos(thetaOrtLeft), Mathf.Sin(thetaOrtLeft));
                    float thetaOrtRight = theta_BA - thetaBAOrt;
                    BA.rightBound.Set(Mathf.Cos(thetaOrtRight), Mathf.Sin(thetaOrtRight));
                    RVO_All.Add(BA);

                }
                robots[i].setVelocity(intersect(i, RVO_All));
          //  }
        }
            

    }

    private Vector2 intersect(int robIndex, ArrayList RVO_All)
    {
        Robot myRobo = robots[robIndex];
        Vector2 VAPost = Vector2.zero;
        Vector2 newVel = Vector2.zero;
        bool suit = true;
        float normVelo = myRobo.DesriedVelocity.magnitude;
        ArrayList suitableVelo = new ArrayList();
        ArrayList unSuitableVelo = new ArrayList();
        float PI2 = Mathf.PI*2;
        for(float theta = 0f; theta < PI2; theta += 0.1f)
        {
            float velStep = normVelo / 10.0f;
            for(float rad = 0.02f;rad< normVelo + 0.02f; rad += velStep)
            {
                newVel.Set(rad*Mathf.Cos(theta),rad * Mathf.Sin(theta));
                suit = true;
                foreach(RVO_BA BA in RVO_All)
                {
                    Vector2 dif=Vector2.zero;
                    dif.Set(newVel.x + myRobo.transform.position.x - BA.trans.x, newVel.y + myRobo.transform.position.y -BA.trans.y);
                    float theta_diff = Mathf.Atan2(dif.y, dif.x);
                    float theta_right = Mathf.Atan2(BA.rightBound.y, BA.rightBound.x);
                    float theta_left = Mathf.Atan2(BA.leftBound.y, BA.leftBound.x);
                    if (inBetween(theta_right,theta_diff,theta_left))
                    {
                        suit = false;
                        break;
                    }

                }
                if (suit)
                {
                    suitableVelo.Add(newVel);
                }
                else
                {
                    unSuitableVelo.Add(newVel);
                }
            }
        }
        newVel = myRobo.DesriedVelocity;
        suit = true;
        foreach (RVO_BA BA in RVO_All)
        {
            Vector2 dif = Vector2.zero;
            dif.Set(newVel.x + myRobo.transform.position.x - BA.trans.x, newVel.y + myRobo.transform.position.y - BA.trans.y);
            float theta_diff = Mathf.Atan2(dif.y, dif.x);
            float theta_right = Mathf.Atan2(BA.rightBound.y, BA.rightBound.x);
            float theta_left = Mathf.Atan2(BA.leftBound.y, BA.leftBound.x);
            if (inBetween(theta_right, theta_diff, theta_left))
            {
                suit = false;
                break;
            }

        }
        if (suit)
        {
            
            suitableVelo.Add(newVel);
        }
        else
        {
            unSuitableVelo.Add(newVel);
        }

        if(suitableVelo.Count > 0)
        {
            Debug.Log(myRobo.name + " has " + suitableVelo.Count + " suitable velo");
            VAPost = min(suitableVelo, myRobo.DesriedVelocity);

        }
        else
        {
            IDictionary<Vector2,float> tc_V = new Dictionary<Vector2, float>();
            foreach(Vector2 unsuitV in unSuitableVelo)
            {
                tc_V[unsuitV] = 0;
                ArrayList tc = new ArrayList();
               
                foreach (RVO_BA BA in RVO_All)
                {
                    Vector2 dif = Vector2.zero;
                    float rad = BA.avoidRadius;
                    dif.Set(unsuitV.x + myRobo.transform.position.x - BA.trans.x, unsuitV.y + myRobo.transform.position.y - BA.trans.y);
                    float theta_dif = Mathf.Atan2(dif.y, dif.x);
                    float theta_right = Mathf.Atan2(BA.rightBound.y, BA.rightBound.x);
                    float theta_left= Mathf.Atan2(BA.leftBound.y, BA.leftBound.x);
                    if (inBetween(theta_right, theta_dif, theta_left))
                    {
                        float small_theta = Mathf.Abs(theta_dif -0.5f*(theta_left+ theta_right));
                        float temp = Mathf.Abs(BA.dist * Mathf.Sin(small_theta));
                        if (temp >= rad)
                        {
                            rad = temp;
                        }
                        float big_theta = Mathf.Asin(Mathf.Abs(BA.dist * Mathf.Sin(small_theta)) / rad);
                        float dist_tg = Mathf.Abs(BA.dist * Mathf.Cos(small_theta)) - Mathf.Abs(rad * Mathf.Cos(big_theta));
                        if(dist_tg < 0)
                        {
                            dist_tg = 0;
                        }
                        float tc_v = dist_tg / dif.magnitude;
                        tc.Add(tc_v);
                    }

                }
                tc_V[unsuitV] = min(tc) + 0.001f;
            }
            float WT = 0.2f;
            VAPost = (Vector2)unSuitableVelo[0];
            float lastKey = 0f;
            foreach (Vector2 v in unSuitableVelo)
            {
                
                Vector2 temp = v - myRobo.DesriedVelocity;
                float key = ((WT / tc_V[v])) + temp.magnitude;
                if (!VAPost.Equals(v))
                {
                    if (key< lastKey)
                    {
                        lastKey = key;
                        VAPost = v;
                    }
                }else
                {
                    lastKey = key;
                    VAPost = v;
                }
            }
        }
        return VAPost;
    }

    private bool inBetween(float thetaRight, float thetaDif,float thetaLeft)
    {
        if(Mathf.Abs(thetaRight- thetaLeft)<= Mathf.PI)
        {
            if(thetaRight<=thetaDif && thetaDif<= thetaLeft)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            if(thetaLeft <0 && thetaRight> 0)
            {
                thetaLeft += 2 * Mathf.PI;
                if (thetaDif < 0)
                {
                    thetaDif += 2 * Mathf.PI;
                }
                if (thetaRight <= thetaDif && thetaDif <= thetaLeft)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            if (thetaLeft > 0 && thetaRight< 0)
            {
                thetaRight += 2 * Mathf.PI;
                if (thetaDif < 0)
                {
                    thetaDif += 2 * Mathf.PI;
                }
                if (thetaLeft <= thetaDif && thetaDif <= thetaRight )
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        return true;
    }
    private Vector2 min(ArrayList mylist, Vector2 desVel)
    {
        Vector2 result = -desVel;
        float diff =2*desVel.magnitude;
        foreach(Vector2 vel in mylist)
        {
            Vector2 diffVec = vel - desVel;
            if (diffVec.magnitude < diff)
            {
                result = vel;
                diff = diffVec.magnitude;
            }
        }
        return result;
    }
    private float min(ArrayList mylist)
    {
        
        float diff = (float)mylist[0];
        foreach (float vel in mylist)
        {
            
            if (vel < diff)
            {
                diff = vel;
            
            }
        }
        return diff;
    }
}

struct RVO_BA
{
    public Vector2 trans;
    public Vector2 leftBound;
    public Vector2 rightBound;
    public float dist;
    public float avoidRadius;
}