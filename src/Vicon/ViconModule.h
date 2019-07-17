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


/**
 * @ingroup driver_components
 * @file
 * Vicon driver
 * This file contains the driver component to
 * talk to the Vicon infrared tracking system.
 *
 * The driver is build from one module to handle
 * the sockets communicationa and compontens for
 * each tracked object.
 *
 * The received data is sent via a push interface.
 *
 * @author Manuel Huber <huberma@in.tum.de>
 */
#ifndef __ViconModule_h_INCLUDED__
#define __ViconModule_h_INCLUDED__

#include <string>
#include <cstdlib>

#include "ViconDataStreamSDK_CPP/Client.h"

#include <iostream>
#include <map>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/utility.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/Module.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>

namespace Ubitrack { namespace Drivers {
using namespace Dataflow;

// forward declaration
class ViconComponent;
class ViconRigidBodyReceiverComponent;
class ViconPointCloudReceiverComponent;

/**
 * Module key for vicon.
 * Represents the port number on which to listen.
 */
MAKE_NODEATTRIBUTEKEY_DEFAULT( ViconModuleKey, std::string, "ViconTracker", "serverName", "ViconTracker.local" );


/**
 * Component key for vicon.
 * Represents the body number
 */
class ViconComponentKey
{
public:
    enum TargetType { target_6d, target_3dcloud };

	// still ugly refactor vicon driver sometime..
	// construct from configuration
	ViconComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: m_name( "" )
	, m_targetType( target_6d )
	{
		Graph::UTQLSubgraph::EdgePtr config;

	  if ( subgraph->hasEdge( "Output" ) )
		  config = subgraph->getEdge( "Output" );

	  if ( !config )
	  {
		  UBITRACK_THROW( "ViconTracker Pattern has no \"Output\" edge");
	  }

	  config->getAttributeData( "viconBodyName", m_name );

	  if (( m_name.empty() ))
            UBITRACK_THROW( "Missing or invalid \"viconBodyName\" attribute on \"Output\" edge" );


	  // type of the component
	  std::string typeString = subgraph->m_DataflowAttributes.getAttributeString( "viconType" );
	  if ( typeString.empty() )
	  {
	      // no explicit vicon target type information. so we assume 6D
	      m_targetType = target_6d;
	  }
	  else
	  {
	      if ( typeString == "6d" )
			  m_targetType = target_6d;
	      else if ( typeString == "3dcloud" )
		  {
			  m_targetType = target_3dcloud;
		  }
	      else
			  UBITRACK_THROW( "Vicon target with unknown target type: " + typeString );
	  }

	}

	// construct from body number
	ViconComponentKey( std::string name )
		: m_name( name )
        , m_targetType( target_6d )
 	{}

    // construct from body number and target type
    ViconComponentKey( std::string name, TargetType t )
        : m_name( name )
        , m_targetType( t )
    {}

	std::string getName() const
	{
		return m_name;
	}

    TargetType getTargetType() const
    {
        return m_targetType;
    }

	// less than operator for map
	bool operator<( const ViconComponentKey& b ) const
    {
        if ( m_targetType == b.m_targetType )
			return m_name.compare(b.m_name) < 0;
        else
            return m_targetType < b.m_targetType;
    }

protected:
	std::string m_name;
	TargetType m_targetType;
};


/**
 * Module for Vicon tracker.
 * Does all the work
 */
class ViconModule
	: public Module< ViconModuleKey, ViconComponentKey, ViconModule, ViconComponent >
{
public:
	/** UTQL constructor */
	ViconModule( const ViconModuleKey& key, boost::shared_ptr< Graph::UTQLSubgraph >, FactoryHelper* pFactory );

	/** destructor */
	~ViconModule();

	virtual void startModule();

	virtual void stopModule();

	void processSubjects(Ubitrack::Measurement::Timestamp ts);
	void processMarkers(Ubitrack::Measurement::Timestamp ts);

	inline long int getDefaultLatency() {
		return m_defaultLatency;
	}

protected:
	bool CreateClient();
	void ThreadProc();

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;
	
	Measurement::TimestampSync m_synchronizer;

	/** Timestamp of the last received measurement */
	Ubitrack::Measurement::Timestamp m_lastTimestamp;

	/** create the components **/
	boost::shared_ptr< ComponentClass > createComponent( const std::string&, const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph> subgraph,
				const ComponentKey& key, ModuleClass* pModule );



private:

	int CreateClient(int iConnectionType);
	ViconDataStreamSDK::CPP::Client* theClient;

    std::string m_serverName;
    std::string m_clientName;

	long int m_defaultLatency;
};

std::ostream& operator<<( std::ostream& s, const ViconComponentKey& k );

/**
 * Component for Vicon tracker.
 * Does nothing but provide a push port

 * @TODO: make this two separate components for 6d/3dlist
 */
class ViconComponent : public ViconModule::Component {
public:
	/** constructor */
	ViconComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const ViconComponentKey& componentKey, ViconModule* pModule )
		: ViconModule::Component( name, componentKey, pModule )
		, m_latencyPort("Latency", *this, boost::bind( &ViconComponent::receiveLatency, this, _1 ) )
		, m_latency(pModule->getDefaultLatency())
	{
	}

	template< class EventType >
	void send( const EventType& rEvent ) {
		UBITRACK_THROW("Not Implemented.");
	};

	void receiveLatency( const Measurement::Distance& m );
	
	/** destructor */
	~ViconComponent();

protected:
	PushConsumer< Ubitrack::Measurement::Distance > m_latencyPort;
	long int m_latency;

};

class ViconRigidBodyReceiverComponent : public ViconComponent {
public:
	/** constructor */
	ViconRigidBodyReceiverComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const ViconComponentKey& componentKey, ViconModule* pModule )
		: ViconComponent( name, subgraph, componentKey, pModule )
		, m_port( "Output", *this )
	{}
	
	inline void send( const Ubitrack::Measurement::Pose& rEvent ) {
		m_port.send(Ubitrack::Measurement::Pose(rEvent.time() -  m_latency, rEvent));
	};

protected:
	// output port
	PushSupplier< Ubitrack::Measurement::Pose > m_port;
};

class ViconPointCloudReceiverComponent : public ViconComponent {
public:
	/** constructor */
	ViconPointCloudReceiverComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const ViconComponentKey& componentKey, ViconModule* pModule )
		: ViconComponent( name, subgraph, componentKey, pModule )
		, m_port( "Output", *this )
	{}

	inline void send( const Ubitrack::Measurement::PositionList& rEvent ) {
		m_port.send(Ubitrack::Measurement::PositionList(rEvent.time() -  m_latency, rEvent));
	};

protected:
	// output port
	PushSupplier< Ubitrack::Measurement::PositionList > m_port;
};


} } // namespace Ubitrack::Drivers

#endif
