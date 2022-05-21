# InterpolateDict equivalent with FRC254's InterpolatingTreeMap:
# https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/lib/util/InterpolatingDouble.java

import bisect

class InterpolateDict(dict):
    def getInterpolated(self, k):
        gotVal = self.get(k)

        if gotVal is None:
            keys = sorted(self)
            index = bisect.bisect(keys, k)
            topBound = None
            bottomBound = None

            if index > 0:
                bottomBound = keys[index - 1]
            if index < len(keys):
                topBound = keys[index]

            if topBound is None and bottomBound is None:
                return None
            elif topBound is None:
                return self.get(bottomBound)
            elif bottomBound is None:
                return self.get(topBound)

            topElem = self.get(topBound)
            bottomElem = self.get(bottomBound)

            upper_to_lower = topBound - bottomBound if topBound > bottomBound else 0.0
            query_to_lower = k - bottomBound if k > bottomBound else 0.0

            return (topElem - bottomElem) * (query_to_lower / upper_to_lower) + bottomElem
        else:
            return gotVal
