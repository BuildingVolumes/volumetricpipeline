using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerMovement : MonoBehaviour
{
	CharacterController _charController;
    // Start is called before the first frame update
    void Start()
    {
     	_charController = GetComponent<CharacterController>();   
    }

    // Update is called once per frame
    void Update()
    {
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        var movement = Vector3.forward * vertical + Vector3.right *horizontal;

        _charController.Move(movement * Time.deltaTime);
        
    }
}
