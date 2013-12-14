using UnityEngine;
using System.Collections;

public class SolarSystemObject : MonoBehaviour {
	public SolarSystem SolarSystem;

	public float AngleToSun;

	public virtual void Start() {

	}

	public virtual void Update() {

	}

	public Sun Sun {
		get { return this.SolarSystem.Sun; }
	}
}
