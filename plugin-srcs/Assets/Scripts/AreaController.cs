using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AreaController : MonoBehaviour
{
    public static AreaController instance;
    public GameObject m_Area;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void SetValue()
    {
        Debug.Log("OK");
        Debug.Log(m_Area);
    }
}
