/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */


#include "ViconModule.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <utDataflow/ComponentFactory.h>
#include <utMath/Vector.h>
#include <utMath/Quaternion.h>
#include <utMath/Pose.h>
#include <utMath/Matrix.h>
#include <utUtil/OS.h>
#include <boost/array.hpp>

#include <log4cpp/Category.hh>





namespace Ubitrack { namespace Drivers {

namespace Vicon = ViconDataStreamSDK::CPP;

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.Vicon" ) );

std::string Adapt(const Vicon::Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
    case Vicon::Direction::Forward:
      return "Forward";
    case Vicon::Direction::Backward:
      return "Backward";
    case Vicon::Direction::Left:
      return "Left";
    case Vicon::Direction::Right:
      return "Right";
    case Vicon::Direction::Up:
      return "Up";
    case Vicon::Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

std::string Adapt(const Vicon::Result::Enum i_result)
{
  switch (i_result)
  {
    case Vicon::Result::ClientAlreadyConnected:
      return "ClientAlreadyConnected";
    case Vicon::Result::ClientConnectionFailed:
      return "";
    case Vicon::Result::CoLinearAxes:
      return "CoLinearAxes";
    case Vicon::Result::InvalidDeviceName:
      return "InvalidDeviceName";
    case Vicon::Result::InvalidDeviceOutputName:
      return "InvalidDeviceOutputName";
    case Vicon::Result::InvalidHostName:
      return "InvalidHostName";
    case Vicon::Result::InvalidIndex:
      return "InvalidIndex";
    case Vicon::Result::InvalidLatencySampleName:
      return "InvalidLatencySampleName";
    case Vicon::Result::InvalidMarkerName:
      return "InvalidMarkerName";
    case Vicon::Result::InvalidMulticastIP:
      return "InvalidMulticastIP";
    case Vicon::Result::InvalidSegmentName:
      return "InvalidSegmentName";
    case Vicon::Result::InvalidSubjectName:
      return "InvalidSubjectName";
    case Vicon::Result::LeftHandedAxes:
      return "LeftHandedAxes";
    case Vicon::Result::NoFrame:
      return "NoFrame";
    case Vicon::Result::NotConnected:
      return "NotConnected";
    case Vicon::Result::NotImplemented:
      return "NotImplemented";
    case Vicon::Result::ServerAlreadyTransmittingMulticast:
      return "ServerAlreadyTransmittingMulticast";
    case Vicon::Result::ServerNotTransmittingMulticast:
      return "ServerNotTransmittingMulticast";
    case Vicon::Result::Success:
      return "Success";
    case Vicon::Result::Unknown:
      return "Unknown";
    default:
      return "unknown";
  }
}





ViconModule::ViconModule( const ViconModuleKey& moduleKey, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
	: Module< ViconModuleKey, ViconComponentKey, ViconModule, ViconComponent >( moduleKey, pFactory )
	, m_synchronizer( 100 ) // assume 100 Hz for timestamp synchronization
    , m_serverName(m_moduleKey.get())
    , m_clientName("")
	, m_defaultLatency(10000000)
    , m_lastTimestamp(0)
	, theClient(NULL)
{

	Graph::UTQLSubgraph::NodePtr config;

	if ( subgraph->hasNode( "ViconTracker" ) )
	  config = subgraph->getNode( "ViconTracker" );

	if ( !config )
	{
	  UBITRACK_THROW( "ViconTracker Pattern has no \"ViconTracker\" node");
	}

	m_clientName = config->getAttributeString( "clientName" );
	config->getAttributeData("latency", m_defaultLatency);

}



// Establish a Vicon Client connection
bool ViconModule::CreateClient()
{
    // release previous server
    if(theClient)
    {
        if (theClient->IsConnected().Connected) {
        	theClient->Disconnect();
        }
        delete theClient;
    }

    // create Vicon client
    theClient = new Vicon::Client();

    // print version info
    Vicon::Output_GetVersion ver = theClient->GetVersion();
    LOG4CPP_INFO(logger, "Vicon Tracker via Datastream SDK (ver. " << ver.Major << "." << ver.Minor << "." << ver.Point << ")");

    int maxTries = 5;
	int tryCount = 0;
	bool clientJustConnected = false;
	while( !theClient->IsConnected().Connected && tryCount < maxTries)
	{
		LOG4CPP_INFO(logger, "Connecting to Vicon server host" << m_serverName << ", try " << (tryCount+1) << "");
		// Alternatively we could use a Multicast connection
		theClient->Connect( m_serverName.c_str() );
		clientJustConnected = theClient->IsConnected().Connected;
		tryCount++;
		Util::sleep(1000);
	}

	if (tryCount == maxTries) {
		LOG4CPP_ERROR( logger, "Error connecting to Vicon Server.");
		return false;
	}


    // enable the data we want to read
    theClient->EnableMarkerData();
    //theClient->EnableUnlabeledMarkerData();
    theClient->EnableSegmentData();
    if (!theClient->IsMarkerDataEnabled().Enabled || !theClient->IsSegmentDataEnabled().Enabled) {
    	LOG4CPP_ERROR( logger, "Marker or Segment data stream is not active.");
    }

    // set the stream mode
    //theClient->SetStreamMode( Vicon::StreamMode::ClientPull );
    theClient->SetStreamMode( Vicon::StreamMode::ServerPush );
  
    // Set the global up axis
    theClient->SetAxisMapping(Vicon::Direction::Right, 
                              Vicon::Direction::Up, 
                              Vicon::Direction::Backward ); // Y-up



	Vicon::Output_GetAxisMapping _Output_GetAxisMapping = theClient->GetAxisMapping();
	LOG4CPP_INFO(logger, "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) << " Y-"
        << Adapt(_Output_GetAxisMapping.YAxis) << " Z-" << Adapt(_Output_GetAxisMapping.ZAxis) );

	return true;
}



void ViconModule::startModule()
{
	LOG4CPP_INFO( logger, "Creating ViconTracker for server: " << m_moduleKey.get() );

	// Create Vicon Client
    if(!CreateClient())
    {
        LOG4CPP_ERROR(logger, "Error initializing client.  See log for details.  Exiting");
        return;
    }
    else
    {
        LOG4CPP_INFO(logger, "Client initialized and ready.");
    }

 	LOG4CPP_TRACE( logger, "Starting thread..." );

	if ( !m_running )
	{
		m_bStop = false;
		m_Thread.reset( new boost::thread( boost::bind ( &ViconModule::ThreadProc, this ) ) );
		m_running = true;
	}

}


void ViconModule::stopModule()
{
	LOG4CPP_INFO( logger, "Stopping Vicon network service: " << m_moduleKey.get() );
	//Module<ViconModuleKey, ViconComponentKey, ViconModule, ViconComponent>::stopModule();

 	LOG4CPP_TRACE( logger, "Stopping thread..." );

	if ( m_running )
	{
		LOG4CPP_TRACE( logger, "Thread was running" );
		m_running = false;
		if ( m_Thread )
		{
			m_bStop = true;
			m_Thread->join();
		}

	}
	if (theClient) {

		LOG4CPP_INFO(logger, "Uninitialize ViconClient: " << m_moduleKey.get());
		theClient->Disconnect();
		Util::sleep(50); // wait 50ms after uninitialize ..
		delete theClient;
		theClient = NULL;
	}
}


ViconModule::~ViconModule()
{
	// delete references
	if (theClient) {
		stopModule();
		if (theClient) {
			delete theClient;	
			theClient = NULL;
		}
	}

}

void ViconModule::ThreadProc() {

	while ( !m_bStop )
	{
		int frameResult = theClient->GetFrame().Result;
		if (frameResult == Vicon::Result::Success) {

			Ubitrack::Measurement::Timestamp timestamp = Ubitrack::Measurement::now();

			m_lastTimestamp = timestamp;

			// stats about the frame
			int frameNumber = theClient->GetFrameNumber().FrameNumber;
			double frameLatency = theClient->GetLatencyTotal().Total;


			// use synchronizer to correct timestamps
			// XXX is this correct ??
			//timestamp = m_synchronizer.convertNativeToLocal( data->fTimestamp, timestamp );


			//LOG4CPP_DEBUG( logger , "Vicon Latency: " << frameLatency );
			// should substract latency + network from timestamp .. instead of constant.
			// was 19,000,000
			//timestamp -= frameLatency / XXXX CONVERT FROM SECONDS...;

			if (m_running) {
				processSubjects(timestamp);
				processMarkers(timestamp);
			}

		}
		else {
			LOG4CPP_DEBUG( logger, "Error receiving frame.");
			Util::sleep(100);
		}
	}

}

void ViconModule::processSubjects(const Ubitrack::Measurement::Timestamp timestamp) {
	std::string tracked_frame, subject_name, segment_name;
    unsigned int n_subjects = theClient->GetSubjectCount().SubjectCount;
    static unsigned int cnt = 0;

    for (unsigned int i_subjects = 0; i_subjects < n_subjects; i_subjects++)
    {

      subject_name = theClient->GetSubjectName(i_subjects).SubjectName;
      unsigned int n_segments = theClient->GetSegmentCount(subject_name).SegmentCount;

      for (unsigned int i_segments = 0; i_segments < n_segments; i_segments++)
      {
        segment_name = theClient->GetSegmentName(subject_name, i_segments).SegmentName;

        Vicon::Output_GetSegmentGlobalTranslation trans = theClient->GetSegmentGlobalTranslation(subject_name, segment_name);
        Vicon::Output_GetSegmentGlobalRotationQuaternion quat = theClient->GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
        if (trans.Result == Vicon::Result::Success && quat.Result == Vicon::Result::Success)
        {
          if (!trans.Occluded && !quat.Occluded)
		  {
			  //tracked_frame = subject_name + "/" + segment_name;

			  tracked_frame = subject_name;
			  ViconComponentKey key( tracked_frame, ViconComponentKey::target_6d );

			  // check for tracker component
			  if ( hasComponent( key ) )
			  {
				  //LOG4CPP_INFO(logger, "Segment name : "<< tracked_frame);
				  // generate pose
				  Ubitrack::Measurement::Pose pose( timestamp,
					  Ubitrack::Math::Pose(
					  Ubitrack::Math::Quaternion((double)quat.Rotation[0], (double)quat.Rotation[1], (double)quat.Rotation[2], (double)quat.Rotation[3]),
		        		Ubitrack::Math::Vector< double, 3 >((double)(trans.Translation[0]/1000.0), (double)(trans.Translation[1]/1000.0), (double)(trans.Translation[2]/1000.0))
//=======
//			if (!trans.Occluded && !quat.Occluded)
//			{
//				tracked_frame = subject_name;
//				//tracked_frame = subject_name + "/" + segment_name;
//
//				//LOG4CPP_INFO(logger, "Key" << segment_name);
//
//				ViconComponentKey key(tracked_frame, ViconComponentKey::target_6d);
//			//	LOG4CPP_INFO(logger, "Test ViconComponentKey : " << key.getName() << " Type: " << key.getTargetType()<<" Have key :" << hasComponent(key));
//
//		    // check for tracker component
//		    if ( hasComponent( key ) )
//		    {
//				// generate pose
//
//				Ubitrack::Measurement::Pose pose(timestamp,
//					Ubitrack::Math::Pose(
//		        		Ubitrack::Math::Quaternion((double)quat.Rotation[0], (double)quat.Rotation[1], (double)quat.Rotation[2], (double)quat.Rotation[3]),
//		        		Ubitrack::Math::Vector< double, 3 >((double)trans.Translation[0], (double)trans.Translation[1], (double)trans.Translation[2])
//>>>>>>> 66e0b33e003398a71b608004029886ba0c4db137
	        		)
		        );

		        //send it to the component
				  //LOG4CPP_INFO(logger, "Frame number: " << theClient->GetFrameNumber().FrameNumber);
				LOG4CPP_DEBUG( logger, "Sending pose for " << tracked_frame << " using " << getComponent( key )->getName() << ": " << pose );
				static_cast<ViconRigidBodyReceiverComponent*>(getComponent( key ).get())->send( pose );
		    }
			else {
				//LOG4CPP_INFO( logger, "Sending pose for " << tracked_frame << " using " << getComponent( key )->getName() );
				LOG4CPP_TRACE(logger, "No component for body " << tracked_frame);
			}
		  }
          else
          {
            if (cnt % 100 == 0) {
              LOG4CPP_DEBUG( logger, "" << subject_name << " occluded, not publishing... " );
			}
          }
        }
        else
        {
          LOG4CPP_DEBUG( logger, "GetSegmentGlobalTranslation/Rotation failed (result = " << Adapt(trans.Result) << ", " << Adapt(quat.Result) << "), not publishing...");
        }
      }
    }
    cnt++;
}

void ViconModule::processMarkers(const Ubitrack::Measurement::Timestamp timestamp) {
	unsigned int n_markers = 0;

	// Count the number of subjects
	unsigned int SubjectCount = theClient->GetSubjectCount().SubjectCount;
	// Get labeled markers
	for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
	{
		std::string this_subject_name = theClient->GetSubjectName(SubjectIndex).SubjectName;
		// Count the number of markers
		unsigned int num_subject_markers = theClient->GetMarkerCount(this_subject_name).MarkerCount;
		n_markers += num_subject_markers;
		//std::cout << "    Markers (" << MarkerCount << "):" << std::endl;

		// handle pointcloud receivers
	    ViconComponentKey key( this_subject_name, ViconComponentKey::target_3dcloud );

	    // check for component
	    if ( hasComponent( key ) )
	    {

	    	// possible without copying ??
			boost::shared_ptr< std::vector< Ubitrack::Math::Vector< double, 3 > > > cloud(new std::vector< Ubitrack::Math::Vector< double, 3 > >(num_subject_markers));
			for (unsigned int MarkerIndex = 0; MarkerIndex < num_subject_markers; ++MarkerIndex)
			{
				std::string this_marker_name = theClient->GetMarkerName(this_subject_name, MarkerIndex).MarkerName;
				//std::string this_segment_name = theClient->GetMarkerParentName(this_subject_name, this_marker.marker_name).SegmentName;

				// Get the global marker translation
				Vicon::Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
				  theClient->GetMarkerGlobalTranslation(this_subject_name, this_marker_name);

				cloud->at(MarkerIndex) = Ubitrack::Math::Vector< double, 3 >(
					_Output_GetMarkerGlobalTranslation.Translation[0]/1000.0,
					_Output_GetMarkerGlobalTranslation.Translation[1]/1000.0, 
					_Output_GetMarkerGlobalTranslation.Translation[2]/1000.0);

				//this_marker.occluded = _Output_GetMarkerGlobalTranslation.Occluded;
			}

			Ubitrack::Measurement::PositionList pc( timestamp, cloud );

	        //send it to the component
			LOG4CPP_DEBUG( logger, "Sending pose for " << this_subject_name << " using " << getComponent( key )->getName() << ": " << cloud );
			static_cast<ViconPointCloudReceiverComponent*>(getComponent( key ).get())->send( pc );
	    }
		else {
			LOG4CPP_TRACE( logger, "No component for cloud name " << this_subject_name );
		}
	}
}


boost::shared_ptr< ViconModule::ComponentClass > ViconModule::createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
	const ComponentKey& key, ModuleClass* pModule )
{
	ViconComponentKey::TargetType tt;
	std::string typeString = subgraph->m_DataflowAttributes.getAttributeString("viconType");
	if (typeString.empty()) {
		// no explicit vicon target type information. so we assume 6D
		tt = ViconComponentKey::target_6d;
	} else {
		if (typeString == "6d")
			tt = ViconComponentKey::target_6d;
		else if (typeString == "3dcloud") {
			tt = ViconComponentKey::target_3dcloud;
		} else
			UBITRACK_THROW(
					"Vicon target with unknown target type: "
							+ typeString);
	}
	if ( tt == ViconComponentKey::target_6d ) {
		return boost::shared_ptr< ComponentClass >( new ViconRigidBodyReceiverComponent( name, subgraph, key, pModule ) );
	} else {
		return boost::shared_ptr< ComponentClass >( new ViconPointCloudReceiverComponent( name, subgraph, key, pModule ) );
	}

}


void ViconComponent::receiveLatency( const Measurement::Distance& m ) {
		double l = *m;		     
		LOG4CPP_DEBUG( logger , "ViconComponent received new latency measurement in ms: " << l );
		// convert ms to timestamp offset
		m_latency = (long int)(1000000.0 * l);

};


ViconComponent::~ViconComponent()
{
	LOG4CPP_INFO( logger, "Destroying Vicon component" );
}

std::ostream& operator<<( std::ostream& s, const ViconComponentKey& k )
{
        s << "ViconComponent[ " << k.getName() << " "
                             << k.getTargetType() << " ]";
        return s;
}


// register module at factory
UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) {
	cf->registerModule< ViconModule > ( "ViconTracker" );
}

} } // namespace Ubitrack::Drivers
