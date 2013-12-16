using System.Linq;
using UnityEngine;
using System.Collections;

public class PlanetDetails : MonoBehaviour {
	private Planet planet;

	private GameObject planetDetailsBackground;
	private TextMesh planetName;
	private GameObject planetDetailsContent;

	public void Awake() {
		this.planet = this.transform.parent.GetComponent<Planet>();
	}

	public void Start () {
		// Fill details info
		this.planetDetailsContent = this.transform.Find("DetailsContent").gameObject;
		this.planetDetailsBackground = this.transform.Find("Background").gameObject;
		var guiTextElements = this.GetComponentsInChildren<TextMesh>();
		this.planetName = guiTextElements.First(gte => gte.name == "PlanetName");
		guiTextElements.First(gte => gte.name == "PlanetName").text = this.planet.Details.Name;
		guiTextElements.First(gte => gte.name == "TypeValue").text = this.planet.Details.Type.ToString();
		guiTextElements.First(gte => gte.name == "ResourcesValue").text = this.planet.Details.Resources;
		guiTextElements.First(gte => gte.name == "PopulationValue").text = this.planet.Details.Population.ToString();
		guiTextElements.First(gte => gte.name == "SpeedValue").text = this.planet.Details.Speed.ToString();
		guiTextElements.First(gte => gte.name == "SpeedValueTime").text = this.planet.Details.Speed.ToString();
		this.planetDetailsContent.SetActive(false);
	}

	public void Update() {
		// Hide when not selected
		if (this.planet.IsSelected) {
			this.planetDetailsContent.SetActive(true);
			this.planetDetailsBackground.SetActive(true);
		}
		else {
			this.planetDetailsContent.SetActive(false);
			this.planetDetailsBackground.SetActive(false);
		}
	}
}
