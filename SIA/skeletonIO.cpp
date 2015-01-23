#include <sstream>

#include "skeletonIO.h"

using namespace std;

Skeleton* readHierarchy(std::ifstream &inputfile) {
	string word;
	// Find the ROOT key word
	inputfile >> word;
	size_t foundKeyWord;
	foundKeyWord = word.find("ROOT");
	while (foundKeyWord == string::npos) {
		inputfile >> word;
		foundKeyWord= word.find("ROOT");
	}
	// Get the ROOT name
	inputfile >> word;
	string rootName = word;
	Skeleton* root;

	// Get all the attributes for ROOT
	float offsetX, offsetY, offsetZ;
	inputfile >> word;
	if (word.find("{") != string::npos) {
		// Get the OFFSET
		inputfile >> word;
		foundKeyWord = word.find("OFFSET");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// OFFSET X
		inputfile >> word;
		istringstream(word) >> offsetX;
		// OFFSET Y
		inputfile >> word;
		istringstream(word) >> offsetY;
		// OFFSET Z
		inputfile >> word;
		istringstream(word) >> offsetZ;

		// Get the CHANNELS
		inputfile >> word;
		foundKeyWord = word.find("CHANNELS");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// Number of channels
		int counter;
		inputfile >> word;
		istringstream(word) >> counter;
		// Get all the channels
		vector<AnimCurve> dofs;
		for (int i = 0; i < counter; i++) {
			inputfile >> word;
			AnimCurve channel;
			channel.name = word;
			dofs.push_back(channel);
		}

		// We can create the root
		root = Skeleton::create(rootName, offsetX, offsetY, offsetZ, NULL);
		root->_dofs = dofs;
		defineRotateOrder(root);

		// Recursively get all the joints
		while (word.find("MOTION") == string::npos) {
			inputfile >> word;
			foundKeyWord = word.find("JOINT");
			if (foundKeyWord != string::npos) {
				readJoint(inputfile, root);
			}
		}

	}

	// The keyword MOTION was read
	readMotion(inputfile, root);

	return root;
}

void readJoint(std::ifstream &inputfile, Skeleton* parent) {
	Skeleton* child;
	string word;
	inputfile >> word;
	size_t foundKeyWord;
	foundKeyWord = word.find("Site");
	if (foundKeyWord == string::npos) {
		// New joint
		string jointName = word;

		// Get all the attributes
		float offsetX, offsetY, offsetZ;
		inputfile >> word;
		if (word.find("{") != string::npos) {
			// Get the OFFSET
			inputfile >> word;
			foundKeyWord = word.find("OFFSET");
			if (foundKeyWord == string::npos) {
				// problem
			}
			// OFFSET X
			inputfile >> word;
			istringstream(word) >> offsetX;
			// OFFSET Y
			inputfile >> word;
			istringstream(word) >> offsetY;
			// OFFSET Z
			inputfile >> word;
			istringstream(word) >> offsetZ;

			// Get the CHANNELS
			inputfile >> word;
			foundKeyWord = word.find("CHANNELS");
			if (foundKeyWord == string::npos) {
				// problem
			}
			// Number of channels
			int counter;
			inputfile >> word;
			istringstream(word) >> counter;
			// Get all the channels
			vector<AnimCurve> dofs;
			for (int i = 0; i < counter; i++) {
				inputfile >> word;
				AnimCurve channel;
				channel.name = word;
				dofs.push_back(channel);
			}

			// We can create the new joint
			child = Skeleton::create(jointName, offsetX, offsetY, offsetZ, parent);
			child->_dofs = dofs;
			defineRotateOrder(child);

			// Recursive calls
			inputfile >> word;
			while (word.find("}") == string::npos) {
				readJoint(inputfile, child);
				inputfile >> word;
			}
		} 
	} else {
		// Terminal case
		string endJoint = "End";
		// Pass "{"
		inputfile >> word;

		// Get the OFFSET
		float offsetX, offsetY, offsetZ;
		inputfile >> word;
		foundKeyWord = word.find("OFFSET");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// OFFSET X
		inputfile >> word;
		istringstream(word) >> offsetX;
		// OFFSET Y
		inputfile >> word;
		istringstream(word) >> offsetY;
		// OFFSET Z
		inputfile >> word;
		istringstream(word) >> offsetZ;

		// We can create the last joint, which has no children
		child = Skeleton::create(endJoint, offsetX, offsetY, offsetZ, parent);

		// Pass the closing bracket
		inputfile >> word;

		// Terminal case, we return
		return;
	}

}

void readMotion(std::ifstream &inputfile, Skeleton* root) {
	string word;
	size_t foundKeyWord;

	// Get the number of frames
	inputfile >> word;
	foundKeyWord = word.find("Frames:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile >> word;
	int nbFrames;
	istringstream(word) >> nbFrames;

	// Get the sampling rate
	inputfile >> word; inputfile >> word;
	foundKeyWord = word.find("Time:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile >> word;
	float sampleRate;
	istringstream(word) >> sampleRate;

	// Read all the frames
	for (int i = 0; i < nbFrames; i++) {
		// The reading begins at the beginning of each word
		readKeyFrame(inputfile, root);
	}
}

void readKeyFrame(std::ifstream &inputfile, Skeleton* skel) {
	string word;
	// Read the values for all the joint's own dofs
	if (!skel->_dofs.empty()) {
		for (size_t i = 0; i < skel->_dofs.size(); i++) {
			inputfile >> word;
			double val;
			istringstream(word) >> val;
			skel->_dofs[i]._values.push_back(val);
		}
	} else {
		// Terminal case : End Site
		return;
	}

	// Recursive call for all its children
	for (Skeleton* child : skel->_children) {
		readKeyFrame(inputfile, child);
	}
}

void defineRotateOrder(Skeleton *skel) {
	vector<AnimCurve> dofs = skel->_dofs;
	size_t i = dofs.size() - 3;
	string nameRot = skel->_dofs[i].name;
	while (i < skel->_dofs.size()) {
		if (!nameRot.compare("Xrotation")) {
			i++;
			nameRot = skel->_dofs[i].name;
			if (!nameRot.compare("Yrotation")) {
				skel->_rorder = roXYZ;
				return;
			}
			if (!nameRot.compare("Zrotation")) {
				skel->_rorder = roXZY;
				return;
			}
		} else if (!nameRot.compare("Yrotation")) {
			i++;
			nameRot = skel->_dofs[i].name;
			if (!nameRot.compare("Xrotation")) {
				skel->_rorder = roYXZ;
				return;
			}
			if (!nameRot.compare("Zrotation")) {
				skel->_rorder = roYZX;
				return;
			}
		} else if (!nameRot.compare("Zrotation")) {
			i++;
			nameRot = skel->_dofs[i].name;
			if (!nameRot.compare("Xrotation")) {
				skel->_rorder = roZXY;
				return;
			}
			if (!nameRot.compare("Yrotation")) {
				skel->_rorder = roZYX;
				return;
			}
		} else {
			i++;
		}
	}
}