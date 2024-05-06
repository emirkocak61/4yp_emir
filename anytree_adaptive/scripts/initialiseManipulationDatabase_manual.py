#!/usr/bin/env python3

import pickle
import rospkg

rp = rospkg.RosPack()
package_path = rp.get_path("anytree_adaptive")

if __name__ == "__main__":
    manipulation_database = {}

    with open(
        package_path
        + "/manipulation_database/manipulation_database.pkl",
        "wb",
    ) as g:
        pickle.dump(manipulation_database, g, protocol=pickle.HIGHEST_PROTOCOL)

    print("SUCCESS: Manipulation Database Reset")
