using System;
using UnityEngine;
using System.Collections;

public class EngineLight : MonoBehaviour {
	private Player player;
	private Light light;

	public void Start () {
		this.player = this.transform.parent.GetComponent<Player>();
		this.light = this.GetComponent<Light>();
	}
	
	public void Update () {
		if (this.player.IsEngineWorking) 
			this.light.intensity = 2f;
		else this.light.intensity = 0;
	}
}
