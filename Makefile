HPP_REPO=https://github.com/humanoid-path-planner
HPP_REPO_FORK=git@github.com:orthez
SRC_DIR=${DEVEL_DIR}/src
#HPP_SERVER=${DEVEL_DIR}/install/bin/hpp-wholebody-step-server
#HPP_SERVER=${DEVEL_DIR}/install/bin/hppcorbaserver
HPP_SERVER=${DEVEL_DIR}/install/bin/hpp-precomputation-corba-server
BUILD_TYPE=Debug
MAKE_CMD=colormake

hpp-motion-prior_branch=master
hpp-motion-prior_repository=${HPP_REPO_FORK}

hpp-corbaserver_branch=master
hpp-corbaserver_repository=${HPP_REPO_FORK}
hpp_ros_branch=master
hpp_ros_repository=${HPP_REPO_FORK}
hpp-wholebody-step_branch=master
hpp-wholebody-step_repository=${HPP_REPO_FORK}
hpp-wholebody-step-corba_branch=master
hpp-wholebody-step-corba_repository=${HPP_REPO_FORK}

all:
	make hpp-corbaserver.install
	make hpp_ros.install
	make hpp-wholebody-step.install
	make hpp-wholebody-step-corba.install

restartserver:
	make hpp-corbaserver.build
	make hpp_ros.build
	make hpp-motion-prior.build
	gdb -ex run ${HPP_SERVER}

%.install: %.update %.build

%.update:
	cd ${DEVEL_DIR}/src/hpp-corbaserver
	cd ${SRC_DIR}/$(@:.update=)
	git remote set-url origin ${$(@:.update=)_repository}/$(@:.update=)
	git pull origin ${$(@:.update=)_branch}
	git submodule update

%.build:
	cd ${SRC_DIR}/$(@:.build=);\
	mkdir -p build;\
	cd ${SRC_DIR}/$(@:.build=)/build;\
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_DIR}/install -DHPP_DEBUG=ON -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..;\
	${MAKE_CMD};\
	${MAKE_CMD} install;
