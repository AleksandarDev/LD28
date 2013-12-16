using System;
using UnityEngine;
using System.Collections;

public class EngineSmoke : MonoBehaviour {
	private Player player;
	private ParticleSystem smoke;

	public void Start() {
		this.player = this.transform.parent.GetComponent<Player>();
		this.smoke = this.GetComponent<ParticleSystem>();
	}

	public void Update() {
		if (this.player.IsEngineWorking) {
			this.smoke.emissionRate = 1000;
		}
		else {
			this.smoke.emissionRate = 0;
		}
	}
}
