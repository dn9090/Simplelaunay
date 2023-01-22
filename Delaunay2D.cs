using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Numerics;

namespace Simplelaunay
{
	internal struct Triangle
	{
		public int a, b, c;

		public Triangle(int a, int b, int c)
		{
			this.a = a;
			this.b = b;
			this.c = c;
		}
	}

	internal struct Edge
	{
		public int a, b;

		public Edge(int a, int b)
		{
			this.a = a;
			this.b = b;
		}
	}

	public static class Delaunay2D
	{
		public static int GetTriangleBufferSize(int vertexCount)
		{
			// Normally the maximum triangle count is 2 * n - 5 but we need
			// some additional space for our super triangle.
			return 2 * vertexCount + 1;
		}

		/// <summary>
		/// Computes triangles based on the provided vertices.
		/// </summary>
		/// <param name="vertices">The vertices.</param>
		/// <returns>The triangles as index triplets of the vertices.</returns>
		public static Span<int> Triangulate(ReadOnlySpan<Vector2> vertices)
		{
			var triangles     = new int[GetTriangleBufferSize(vertices.Length) * 3];
			var triangleCount = Triangulate(vertices, triangles);
			
			return triangles.AsSpan(0, triangleCount * 3);
		}

		/// <summary>
		/// Computes triangles based on the provided vertices.
		/// </summary>
		/// <param name="vertices">The vertices.</param>
		/// <param name="triangles">The triangle buffer.</param>
		/// <returns>The number of triangles.</returns>
		public static int Triangulate(ReadOnlySpan<Vector2> vertices, Span<int> triangles)
		{
			if(triangles.Length * 3 < GetTriangleBufferSize(triangles.Length))
				throw new ArgumentOutOfRangeException(nameof(triangles));
			
			return Triangulate(vertices, MemoryMarshal.Cast<int, Triangle>(triangles));
		}

		internal static int Triangulate(ReadOnlySpan<Vector2> vertices, Span<Triangle> triangles)
		{
			var triangleCount = 0;
			var vertexCount   = vertices.Length;
		
			var min = vertices[0];
			var max = vertices[0];

			for(int i = 0; i < vertexCount; ++i)
			{
				min.X = MathF.Min(min.X, vertices[i].X);
				min.Y = MathF.Min(min.Y, vertices[i].Y);

				max.X = MathF.Max(max.X, vertices[i].X);
				max.Y = MathF.Max(max.Y, vertices[i].Y);
			}

			var mid      = (min + max) / 2f;
			var delta    = max - min;
			var deltaMax = MathF.Max(delta.X, delta.Y);

			// Create the vertices of the super triangle.
			Span<Vector2> superVertices = stackalloc Vector2[] {
				new Vector2(mid.X - 20f * deltaMax, mid.Y - deltaMax),
				new Vector2(mid.X, mid.Y + 20f * deltaMax),
				new Vector2(mid.X + 20f * deltaMax, mid.Y - deltaMax)
			};

			// Construct the super triangle. The indices need to be larger than
			// the vertex count to identify vertices of the super triangle with
			// a simple greater-or-equal test.
			triangles[triangleCount++] = new Triangle(vertexCount, vertexCount + 1, vertexCount + 2);

			var polygon    = new List<Edge>();
			var badIndices = new HashSet<int>();

			var H = 0;

			for(int v = 0; v < vertexCount; ++v)
			{
				for(int t = 0; t < triangleCount; ++t)
				{
					var a = triangles[t].a;
					var b = triangles[t].b;
					var c = triangles[t].c;

					// Check if the vertex belongs to the given vertices, otherwise substract
					// the vertex count and fetch the vertex from the super triangle.
					var va = a < vertexCount ? vertices[a] : superVertices[a - vertexCount];
					var vb = b < vertexCount ? vertices[b] : superVertices[b - vertexCount];
					var vc = c < vertexCount ? vertices[c] : superVertices[c - vertexCount];

					if(InCircumCircle(va, vb, vc, vertices[v]))
					{
						badIndices.Add(t);
						
						polygon.Add(new Edge(triangles[t].a, triangles[t].b));
						polygon.Add(new Edge(triangles[t].b, triangles[t].c));
						polygon.Add(new Edge(triangles[t].c, triangles[t].a));
					}
				}

				triangles.RemoveBadIndices(badIndices, ref triangleCount);
				badIndices.Clear();

				for(int i = 0; i < polygon.Count; ++i)
				for(int j = i + 1; j < polygon.Count; ++j)
				{
					if(Equal(polygon[i], polygon[j]))
					{
						badIndices.Add(i);
						badIndices.Add(j);
					}
				}

				polygon.RemoveBadIndices(badIndices);
				badIndices.Clear();

				// Construct the triangle from the edges of the polygon to
				// the vertex.
				for(int i = 0; i < polygon.Count; ++i)
					triangles[triangleCount++] = new Triangle(polygon[i].a, polygon[i].b, v);

				if(triangleCount > H)
					H = triangleCount;

				polygon.Clear();
			}

			// Remove remaining triangles that have vertices in the super triangle.
			for(int i = 0; i < triangleCount; ++i)
			{
				if(triangles[i].a >= vertexCount
				|| triangles[i].b >= vertexCount
				|| triangles[i].b >= vertexCount)
					badIndices.Add(i);
			}

			triangles.RemoveBadIndices(badIndices, ref triangleCount);

			return triangleCount;
		}

		internal static bool InCircumCircle(Vector2 a, Vector2 b, Vector2 c, Vector2 vertex)
		{
			var ab = a.LengthSquared();
			var cd = b.LengthSquared();
			var ef = c.LengthSquared();

			var ax = a.X;
			var ay = a.Y;
			var bx = b.X;
			var by = b.Y;
			var cx = c.X;
			var cy = c.Y;

			var circum = new Vector2();

			circum.X = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay)) / (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));
			circum.Y = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax)) / (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

			circum /= 2f;

			var radius   = (a - circum).LengthSquared();
			var distance = (vertex - circum).LengthSquared();

			return distance <= radius;
		}

		internal static bool Equal(Edge lhs, Edge rhs)
		{
			return (lhs.a == rhs.a && lhs.b == rhs.b) || (lhs.a == rhs.b && lhs.b == rhs.a);
		}

		internal static void RemoveBadIndices(this Span<Triangle> triangles, HashSet<int> indices, ref int triangleCount)
		{
			var count = 0;

			for(int i = 0; i < triangleCount; ++i)
			{
				if(indices.Contains(i))
					continue;
				triangles[count++] = triangles[i];
			}

			triangleCount = count;
		}

		internal static void RemoveBadIndices(this List<Edge> polygon, HashSet<int> indices)
		{
			var count = 0;

			for(int i = 0; i < polygon.Count; ++i)
			{
				if(indices.Contains(i))
					continue;
				polygon[count++] = polygon[i];
			}

			polygon.RemoveRange(count, polygon.Count - count);
		}
	}
}
