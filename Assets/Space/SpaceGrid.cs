using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Vectrosity;

public class SpaceGrid {
	public AvailableResolutions ActiveResolution;

	private Dictionary<AvailableResolutions, Resolution> resolutions; 


	public SpaceGrid(Material gridMaterial) {
		Resolution.Material = gridMaterial;
		this.resolutions = new Dictionary<AvailableResolutions, Resolution>();

		this.GenerateResolutionGrids();

		this.ActiveResolution = AvailableResolutions.SolarSystem;
	}


	public void Update() {
		this.resolutions[this.ActiveResolution].Update();
	}

	private void GenerateResolutionGrids() {
		this.resolutions.Add(AvailableResolutions.Moon, new Resolution(100, 1));
		this.resolutions.Add(AvailableResolutions.Planet, new Resolution(100, 10));
		this.resolutions.Add(AvailableResolutions.SolarSystem, new Resolution(100, 100));
	}

	public class Resolution {
		public static Material Material;

		public readonly float Scale;
		public readonly int Size;

		public VectorLine HorizontalLines;
		public VectorLine VerticalLines;

		private Color color = new Color(0.1f, 0.1f, 0.1f);
		private Color colorFade = new Color(0, 0, 0, 0);
		private float smoothLimit = 0.4f;


		public Resolution(int size, float scale) {
			this.Size = size;
			this.Scale = scale;

			this.FillGrid();
		}


		public void Update() {
			this.HorizontalLines.Draw3DAuto();
			this.VerticalLines.Draw3DAuto();
		}


		private void FillGrid() {
			float smoothLimitInvert = 1 - this.smoothLimit;
			int pointsCount = this.Size * 5;

			var lineColors = new Color[pointsCount];
			var linePoints = new Vector3[pointsCount];
			int index = 0, colorIndex = 0;
			for (int rowIndex = 0; rowIndex < Size; rowIndex++) {
				if (rowIndex % 2 == 0) {
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, -0.5f * this.Scale);
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, this.Size * this.smoothLimit * this.Scale);
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, this.Size * 0.5f * this.Scale);
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, this.Size * smoothLimitInvert * this.Scale);
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, this.Size * this.Scale);
				}
				else {
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, this.Size * this.Scale);
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, this.Size * smoothLimitInvert * this.Scale);
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, this.Size * 0.5f * this.Scale);
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, this.Size * this.smoothLimit * this.Scale);
					linePoints[index++] = new Vector3(rowIndex * this.Scale, 0, -0.5f * this.Scale);
				}

				this.AddGridColors(rowIndex / (float)this.Size, ref lineColors, ref colorIndex);
			}
			this.BuildVectors(out this.HorizontalLines, linePoints, lineColors);

			linePoints = new Vector3[pointsCount];
			lineColors = new Color[pointsCount];
			index = colorIndex = 0;
			for (int columnIndex = 0; columnIndex < this.Size; columnIndex++) {
				if (columnIndex % 2 == 0) {
					linePoints[index++] = new Vector3(-0.5f * this.Scale, 0, columnIndex * this.Scale);
					linePoints[index++] = new Vector3(this.Size * this.smoothLimit * this.Scale, 0, columnIndex * this.Scale);
					linePoints[index++] = new Vector3(this.Size * 0.5f * this.Scale, 0, columnIndex * this.Scale);
					linePoints[index++] = new Vector3(this.Size * smoothLimitInvert * this.Scale, 0, columnIndex * this.Scale);
					linePoints[index++] = new Vector3(this.Size * this.Scale, 0, columnIndex * this.Scale);
				}
				else {
					linePoints[index++] = new Vector3(this.Size * this.Scale, 0, columnIndex * this.Scale);
					linePoints[index++] = new Vector3(this.Size * smoothLimitInvert * this.Scale, 0, columnIndex * this.Scale);
					linePoints[index++] = new Vector3(this.Size * 0.5f * this.Scale, 0, columnIndex * this.Scale);
					linePoints[index++] = new Vector3(this.Size * this.smoothLimit * this.Scale, 0, columnIndex * this.Scale);
					linePoints[index++] = new Vector3(-0.5f * this.Scale, 0, columnIndex * this.Scale);
				}

				this.AddGridColors(columnIndex / (float)Size, ref lineColors, ref colorIndex);
			}

			this.BuildVectors(out this.VerticalLines, linePoints, lineColors);
		}

		private void BuildVectors(out VectorLine destination, IEnumerable<Vector3> points, IEnumerable<Color> colors) {
			var pointsArray = points.ToArray();
			destination = new VectorLine(
				"SolarSystemGridPart",
				pointsArray,
				Resolution.Material,
				0.8f,
				LineType.Continuous);
			destination.SetColorsSmooth(colors.Take(pointsArray.Length - 1).ToArray());
		}

		private void AddGridColors(float progress, ref Color[] lineColors, ref int colorIndex) {
			if (progress >= this.smoothLimit || progress <= 1 - this.smoothLimit) {
				var smoothValue = Mathf.Min(progress, 1 - progress) / this.smoothLimit;
				lineColors[colorIndex++] = new Color(this.color.r, this.color.g, this.color.b, smoothValue);
				lineColors[colorIndex++] = new Color(this.color.r, this.color.g, this.color.b, smoothValue);
				lineColors[colorIndex++] = new Color(this.color.r, this.color.g, this.color.b, smoothValue);
			}
			else {
				lineColors[colorIndex++] = this.color;
				lineColors[colorIndex++] = this.color;
				lineColors[colorIndex++] = this.color;
			}
			lineColors[colorIndex++] = this.colorFade;
			lineColors[colorIndex++] = this.colorFade;
		}
	}

	public enum AvailableResolutions {
		Moon = 1,
		Planet = 10,
		SolarSystem = 100,
		Galaxy = 10000,
		Space = 1000000
	}
}