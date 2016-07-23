/*
 * LIDAR System Library
 * Copyright (C) 2014-2016  Branislav Holý <branoholy@gmail.com>
 *
 * This file is part of LIDAR System Library.
 *
 * LIDAR System Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LIDAR System Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LIDAR System Library.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "lsl/geom/orthogonalpolygondecomposition.hpp"

namespace lsl {
namespace geom {

Eigen::MatrixXi OrthogonalPolygonDecomposition::decompose(const Eigen::MatrixXi& polygonEdges) const
{
	int directions[] = { -1, 1 };

	Eigen::MatrixXi decomposed(polygonEdges.rows(), polygonEdges.cols());
	decomposed.fill(-1);

	Eigen::MatrixXi neighbours = neighboursCount(polygonEdges);

	int blockId = 0;
	while(Eigen::Vector2i *leader = findLeader(neighbours, 4))
	{
		Eigen::Vector2i corners[] = { *leader, *leader + Eigen::Vector2i(1, 1) };

		while(true)
		{
			std::size_t bestI, bestD, bestSum = 0;
			for(std::size_t i = 0; i < 2; i++)
			{
				for(std::size_t d = 0; d < 2; d++)
				{
					corners[i](d) += directions[i];

					if(corners[i](d) >= 0 && corners[1](0) <= neighbours.rows() && corners[1](1) <= neighbours.cols())
					{
						std::size_t sum = 0;
						bool validRegion = true;
						for(int r = corners[0](0); r < corners[1](0) && validRegion; r++)
						{
							for(int c = corners[0](1); c < corners[1](1); c++)
							{
								if(neighbours(r, c) < 0)
								{
									sum = 0;
									validRegion = false;
									break;
								}
								else
								{
									sum += neighbours(r, c);
								}
							}
						}

						if(sum > bestSum)
						{
							bestSum = sum;
							bestI = i;
							bestD = d;
						}
					}

					corners[i](d) -= directions[i];
				}
			}

			if(bestSum > 0) corners[bestI](bestD) += directions[bestI];
			else break;
		}

		decomposed.block(corners[0](0), corners[0](1), corners[1](0) - corners[0](0), corners[1](1) - corners[0](1)).fill(blockId++);
		neighbours.block(corners[0](0), corners[0](1), corners[1](0) - corners[0](0), corners[1](1) - corners[0](1)).fill(-1);

		delete leader;
	}

	return decomposed;
}

Eigen::MatrixXi OrthogonalPolygonDecomposition::neighboursCount(const Eigen::MatrixXi& polygonEdges) const
{
	Eigen::MatrixXi neighbours(polygonEdges.rows(), polygonEdges.cols());

	for(int r = 0; r < polygonEdges.rows(); r++)
	{
		for(int c = 0; c < polygonEdges.cols(); c++)
		{
			if(polygonEdges(r, c) >= 0)
			{
				int count = 0;
				for(int dr = -1, dc = -1; dr < 2; dr += 2, dc += 2)
				{
					int nr = r + dr;
					int nc = c + dc;

					if(nr >= 0 && nr < polygonEdges.rows() && polygonEdges(nr, c) >= 0) count++;
					if(nc >= 0 && nc < polygonEdges.cols() && polygonEdges(r, nc) >= 0) count++;
				}

				neighbours(r, c) = count;
			}
			else
				neighbours(r, c) = -1;
		}
	}

	return neighbours;
}

Eigen::Vector2i* OrthogonalPolygonDecomposition::findLeader(const Eigen::MatrixXi& neighbours, int threshold) const
{
	Vector2i *leader = nullptr;
	for(int r = 0; r < neighbours.rows(); r++)
	{
		for(int c = 0; c < neighbours.cols(); c++)
		{
			if(neighbours(r, c) < 0)
				continue;

			if(leader == nullptr || neighbours(r, c) > neighbours(leader->at(0), leader->at(1)))
			{
				if(leader == nullptr)
					leader = new Vector2i(-1, -1);

				leader->at(0) = r;
				leader->at(1) = c;

				if(neighbours(r, c) >= threshold)
					return leader;
			}
		}
	}

	return leader;
}

}}
