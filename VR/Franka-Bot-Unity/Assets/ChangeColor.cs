using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChangeColor : MonoBehaviour
{

    [SerializeField] private Material myMaterial;
    private Color targetColor; 

    public void SetColor(Color newColor)
    {
        targetColor = newColor;
        Debug.Log("Color changed to: " + targetColor);
    }

    private void Awake()
    {
        Debug.Log("ChangeColor script is alive on " + gameObject.name);
    }

    // Start is called before the first frame update
    void Start()
    {
        targetColor = Color.red; // Set the default color to red
        myMaterial.color = targetColor;
    }

    // Update is called once per frame
    void Update()
    {
        // targetColor = Color.red;
        myMaterial.color = targetColor;
        Debug.Log("Current color: " + myMaterial.color);
    }
}
