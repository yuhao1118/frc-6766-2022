from wpimath.geometry import Translation2d


class CircleFitter:

    @classmethod
    def fit(cls, radius, points, precision):

        # Find starting point
        xSum, ySum = 0.0, 0.0
        for point in points:
            xSum += point.X()
            ySum += point.Y()
        center = Translation2d(xSum / len(points) + radius, ySum / len(points))

        # Iterate to find optimal center
        shiftDist = radius / 2.0
        minResidual = cls.calcResidual(radius, points, center)

        while True:
            translations = [
                Translation2d(shiftDist, 0.0),
                Translation2d(-shiftDist, 0.0),
                Translation2d(0.0, shiftDist),
                Translation2d(0.0, -shiftDist)
            ]
            bestPoint = center
            centerIsBest = True

            # Check all adjacent positions
            for translation in translations:
                residual = cls.calcResidual(radius, points, center + translation)
                if residual < minResidual:
                    bestPoint = center + translation
                    minResidual = residual
                    centerIsBest = False
                    break

            # Decrease shift, exit, or continue
            if centerIsBest:
                shiftDist /= 2.0
                if shiftDist < precision:
                    return center
            else:
                center = bestPoint

    @classmethod
    def calcResidual(cls, radius, points, center):
        residual = 0.0
        for point in points:
            residual += ((point - center).norm() - radius) ** 2
        return residual
