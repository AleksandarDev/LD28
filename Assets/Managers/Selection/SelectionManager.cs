using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using Assets.Managers.GameEvents;
using UnityEngine;

namespace Assets.Managers.Selection {
	public interface ISelectableGameObject {
		bool Selected(SelectionManager selectionManager);
		bool Deselected(SelectionManager selectionManager);
	}

	public class SelectableGameObject : MonoBehaviour, ISelectableGameObject {
		public bool IsSelected;


		public virtual bool Selected(SelectionManager selectionManager) {
			this.IsSelected = true;
			return true;
		}

		public virtual bool Deselected(SelectionManager selectionManager) {
			this.IsSelected = false;
			return true;
		}
	}

	public class SelectionManager : MonoBehaviour, IManager {
		public DebugOptions Debug = new DebugOptions();

		private GameEventsManager gameEventsManager;
		private readonly List<SelectableGameObject> selectedObjects;


		public SelectionManager() {
			this.selectedObjects = new List<SelectableGameObject>();
		}


		public virtual void Awake() {
			this.gameEventsManager = this.GetComponent<GameEventsManager>();
		}

		public virtual void Start() {
			this.gameEventsManager.OnLeftClick += HandleClick;
		}

		public virtual void Update() { }

		private void HandleClick(GameEventsManager sender, MouseClickEventArgs args) {
			// Create ray of current mouse position on set camera and test for hit
			RaycastHit hit;
			bool didHit = this.GetRayHit(Input.mousePosition, out hit);

			if (didHit) {
				// Get hit object and check if it's selectable
				var selectedObject = hit.transform.gameObject.GetComponent<SelectableGameObject>();
				if (selectedObject != null) {
					if (this.Debug.SelectionMade)
						UnityEngine.Debug.Log(
							String.Format("Mouse down Select on {0}", hit.collider.gameObject.name),
							hit.collider.gameObject);

					// Mark object as selected and add it to list if successfully selected
					var wasSelected = selectedObject.Selected(this);
					if (wasSelected)
						this.selectedObjects.Add(selectedObject);
					else if (this.Debug.SelectionMade)
						UnityEngine.Debug.Log(
							String.Format("Mouse down Refused to select {0}", hit.collider.gameObject.name),
							hit.collider.gameObject);
				}
				else {
					if (this.Debug.SelectionMade)
						UnityEngine.Debug.Log(
							String.Format("Mouse down Invalid selection on {0}", hit.collider.gameObject.name),
							hit.collider.gameObject);
				}
			}
			else {
				// Try to deselect all selected objects
				for (int index = 0; index < this.selectedObjects.Count; index++) {
					if (this.selectedObjects.ElementAt(index).Deselected(this)) {
						this.selectedObjects.RemoveAt(index);
						index--;
					}
				}

				if (this.Debug.SelectionClear)
					UnityEngine.Debug.Log(
						String.Format("Mouse down Selection cleared {0} left", this.selectedObjects.Count));
			}
		}

		private bool GetRayHit(Vector3 position, out RaycastHit hit) {
			var ray = Camera.main.ScreenPointToRay(position);
			return Physics.Raycast(ray, out hit, Mathf.Infinity);
		}


		[Serializable]
		public class DebugOptions {
			public bool SelectionMade;
			public bool SelectionClear;
		}
	}
}
