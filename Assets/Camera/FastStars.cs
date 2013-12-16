using UnityEngine;
using System.Collections;

public class FastStars : MonoBehaviour {
	private ParticleSystem stars;
	private Vector3 lastCameraPosition;


	public void Start () {
		this.stars = this.GetComponent<ParticleSystem>();
	}
	
	public void Update () {
		var currentPosition = Camera.main.transform.position;
		if (currentPosition != this.lastCameraPosition)
			this.stars.emissionRate = 10;
		else this.stars.emissionRate = 0;

		this.lastCameraPosition = currentPosition;
	}
}
