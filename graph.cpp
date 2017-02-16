// Author(s): Rimco Boudewijns and Sjoerd Cranen
// Copyright: see the accompanying file COPYING or copy at
// https://svn.win.tue.nl/trac/MCRL2/browser/trunk/COPYING
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//

#include <unordered_map>

#include <QDomDocument>
#include <QTextStream>
#include <QFile>
#include <QtOpenGL>

#include "graph.h"
#include "mcrl2/lts/probabilistic_lts.h"
#include "mcrl2/lts/lts_io.h"
#include "mcrl2/lts/lts_aut.h"
#include "mcrl2/lts/lts_lts.h"
#include "mcrl2/lts/lts_fsm.h"

namespace Graph
{

  namespace detail
  {

    inline float frand(float min, float max)
    {
      return ((float)qrand() / RAND_MAX) * (max - min) + min;
    }

    class GraphImplBase
    {
      public:

        /**
         * @brief Constructor.
         */
        GraphImplBase() : initialState(0) {}

        /**
         * @brief Destructor.
         */
        virtual ~GraphImplBase() {}

        /**
         * @brief Loads a graph with random positioning for the nodes.
         * @param filename The file which contains the graph.
         * @param min The minimum coordinates for any node.
         * @param max The maximum coordinates for any node.
         */
        virtual void load(const QString& filename, const Coord3D& min, const Coord3D& max) = 0;

        /**
         * @brief Returns true if the label is tau.
         * @param labelindex The index of the label.
         */
        virtual bool is_tau(size_t labelindex) const = 0;

        std::vector<NodeNode> nodes;                  ///< Vector containing all graph nodes.
        std::vector<Node> handles;                    ///< Vector containing all handles.
        std::vector<Edge> edges;                      ///< Vector containing all edges.
        std::vector<LabelString> transitionLabels;    ///< Vector containing all transition label strings.
        std::vector<LabelNode> transitionLabelnodes;  ///< Vector containing all transition label nodes.
        std::vector<QString> stateLabels;             ///< Vector containing all state label strings.
        std::vector<LabelNode> stateLabelnodes;       ///< Vector containing all state label nodes.
        size_t initialState;                          ///< Index of the initial state.

        template <typename label_t>
        QString transitionLabel(const label_t& label)
        {
          return QString::fromStdString(label);
        }

        template <typename label_t, class FSM>
        QString stateLabel(const label_t& label, const FSM&)
        {
          return QString::fromStdString(label);
        }
    };

    template <typename graph_t>
    class GraphImpl : public GraphImplBase
    {
      private:
        graph_t m_graph;

        // This local function adds a probabilistic state to the data structures of this graph.
        // For each probabilistic node with two or more outgoing transitions a new node is drawn.
        // For each probability/state pair a new transition is generated labelled with the probability.
        // The index of the newly generated state is returned.
        // If there is only one state in the probabilistic state, then the index of this new state
        // is returned and no new transition is made. 
        
        size_t add_probabilistic_state(const typename graph_t::probabilistic_state_t& probabilistic_state,
                                       const Coord3D& min, 
                                       const Coord3D& max)
        {
          if (probabilistic_state.size()==1)
          {
            return probabilistic_state.begin()->state();
          }
          else
          {
            // There are multiple probabilistic states. Make a new state
            // with outgoing probabilistic transitions to all states. 
            size_t index_of_the_new_probabilistic_state=nodes.size();
            const bool is_probabilistic=true;
            nodes.push_back(NodeNode(Coord3D(frand(min.x, max.x), frand(min.y, max.y), frand(min.z, max.z)),is_probabilistic));
            stateLabelnodes.push_back(LabelNode(nodes[index_of_the_new_probabilistic_state].pos(),index_of_the_new_probabilistic_state));
            
            // The following map recalls where probabilities are stored in transitionLabels.
            typedef std::map < typename graph_t::probabilistic_state_t::probability_t, size_t> probability_map_t;
            probability_map_t probability_label_indices;
            for(const typename graph_t::probabilistic_state_t::state_probability_pair& p: probabilistic_state)
            {
              // Find an index for the probabilistic label of the outgoing transition of the probabilistic state.
              size_t label_index;
              const typename probability_map_t::const_iterator i=probability_label_indices.find(p.probability());
              if (i==probability_label_indices.end()) // not found
              {
                label_index=transitionLabels.size();
                probability_label_indices[p.probability()]=label_index;
                transitionLabels.push_back(LabelString(false,QString::fromStdString(pp(p.probability()))));
              }
              else
              {
                label_index=i->second;
              }

              edges.push_back(Edge(index_of_the_new_probabilistic_state,p.state()));
              handles.push_back(Node((nodes[index_of_the_new_probabilistic_state].pos() + nodes[p.state()].pos()) / 2.0));
              transitionLabelnodes.push_back(LabelNode((nodes[index_of_the_new_probabilistic_state].pos() + nodes[p.state()].pos()) / 2.0,label_index));
            } 
            return index_of_the_new_probabilistic_state;
          }
        }

      public:
        virtual bool is_tau(size_t labelindex) const
        {
          return transitionLabels[labelindex].is_tau();
        }

        virtual void load(const QString& filename, const Coord3D& min, const Coord3D& max)
        {
          // Remove old graph (if it wasn't deleted yet) and load new one
          m_graph.load(filename.toUtf8().constData());

          // Reserve all auxiliary data vectors
          nodes.reserve(m_graph.num_states());
          edges.reserve(m_graph.num_transitions());
          handles.reserve(m_graph.num_transitions());

          transitionLabels.reserve(m_graph.num_action_labels());
          transitionLabelnodes.reserve(m_graph.num_transitions());

          stateLabels.reserve(m_graph.num_state_labels());
          stateLabelnodes.reserve(m_graph.num_states());

          for (size_t i = 0; i < m_graph.num_state_labels(); ++i)
          {
            stateLabels.push_back(stateLabel(m_graph.state_label(i),m_graph));
          }

          // Position nodes randomly
          for (size_t i = 0; i < m_graph.num_states(); ++i)
          {
            const bool is_not_probabilistic=false;
            nodes.push_back(NodeNode(Coord3D(frand(min.x, max.x), frand(min.y, max.y), frand(min.z, max.z)),is_not_probabilistic));
            stateLabelnodes.push_back(LabelNode(nodes[i].pos(),i));
          }

          // Store string representations of labels
          for (size_t i = 0; i < m_graph.num_action_labels(); ++i)
          {
            transitionLabels.push_back(LabelString(m_graph.is_tau(i),transitionLabel(m_graph.action_label(i))));
          }

          // Assign and position edge handles, position edge labels
          for (size_t i = 0; i < m_graph.num_transitions(); ++i)
          {
            mcrl2::lts::transition& t = m_graph.get_transitions()[i];
            size_t new_probabilistic_state=add_probabilistic_state(m_graph.probabilistic_state(t.to()),min,max);
            edges.push_back(Edge(t.from(),new_probabilistic_state));
            handles.push_back(Node((nodes[t.from()].pos() + nodes[new_probabilistic_state].pos()) / 2.0));
            transitionLabelnodes.push_back(LabelNode((nodes[t.from()].pos() + nodes[new_probabilistic_state].pos()) / 2.0,t.label()));
          }

          /* initialState = m_graph.initial_state(); */
          initialState = add_probabilistic_state(m_graph.initial_probabilistic_state(),min,max);
        }
    };


    template <>
    QString GraphImplBase::transitionLabel<mcrl2::lts::action_label_lts>(const mcrl2::lts::action_label_lts& label)
    {
      return QString::fromStdString(mcrl2::lts::pp(label));
    }
    template <>
    QString GraphImplBase::stateLabel<mcrl2::lts::state_label_lts,mcrl2::lts::probabilistic_lts_lts_t>(
                         const mcrl2::lts::state_label_lts& label,
                         const mcrl2::lts::probabilistic_lts_lts_t&)
    {
      return QString::fromStdString(mcrl2::lts::pp(label));
    }
    template <>
    QString GraphImplBase::stateLabel<mcrl2::lts::state_label_empty,mcrl2::lts::probabilistic_lts_aut_t>(
                         const mcrl2::lts::state_label_empty& /*label*/,
                         const mcrl2::lts::probabilistic_lts_aut_t&)
    {
      return QString("");
    }
    template <>
    QString GraphImplBase::stateLabel<mcrl2::lts::state_label_dot,mcrl2::lts::probabilistic_lts_dot_t>(
                         const mcrl2::lts::state_label_dot& label,
                         const mcrl2::lts::probabilistic_lts_dot_t&)
    {
      return QString::fromStdString(mcrl2::lts::pp(label));
    }
    template <>
    QString GraphImplBase::stateLabel<mcrl2::lts::state_label_fsm,mcrl2::lts::probabilistic_lts_fsm_t>(
                         const mcrl2::lts::state_label_fsm& label, 
                         const mcrl2::lts::probabilistic_lts_fsm_t& lts)
    {
      return QString::fromStdString(lts.state_label_to_string(label));
    }

  }

  class Selection
  {
    public:
      const std::vector<size_t>& nodes;
      const std::vector<size_t>& edges;
    private:
      struct Node
      {
        size_t id;    // index in the complete node list
        size_t index; // index in the selected node list
        size_t outdepth;
        size_t indepth;
        std::vector<size_t> inEdges, outEdges; // by edge id
        size_t count;
        Node()=default;
      };

      struct Edge
      {
        size_t id;    // index in the complete edge list
        size_t index; // index in the selected edge list
        size_t count;
        bool bridge;
        Edge()=default;
      };

      detail::GraphImplBase* m_impl;

      // maps node/edge indices to selection Node/Edge objects
      std::unordered_map<size_t,Node> m_nodes;
      std::unordered_map<size_t,Edge> m_edges;
      // keeps track of node/edge indices in selection
      std::vector<size_t> m_nodeIndices;
      std::vector<size_t> m_edgeIndices;

      void repositionNode(const Node& node)
      {
        // Center the first node placed
        if (m_nodeIndices.size() == 1)
        {
          m_impl->nodes[node.id].pos() = Coord3D(0.0, 0.0, 0.0);
          return;
        }

        Coord3D centroid;
        size_t count = 0;
        for (size_t i = 0; i < node.inEdges.size(); ++i)
        {
          ::Graph::Edge& edge = m_impl->edges[node.inEdges[i]];
          if (m_nodes.count(edge.from()))
          {
            centroid += m_impl->nodes[edge.from()].pos();
            ++count;
          }
        }
        for (size_t i = 0; i < node.outEdges.size(); ++i)
        {
          ::Graph::Edge& edge = m_impl->edges[node.outEdges[i]];
          if (m_nodes.count(edge.to()))
          {
            centroid += m_impl->nodes[edge.to()].pos();
            ++count;
          }
        }

        if (count)
        {
          Coord3D rvec = Coord3D(detail::frand(-1.0, 1.0),
            detail::frand(-1.0, 1.0), detail::frand(-1.0, 1.0));
          rvec *= 50.0 / rvec.size();
          m_impl->nodes[node.id].pos() = centroid / ((GLfloat) count) + rvec;
        }
      }

      void repositionEdge(size_t edgeId)
      {
        Coord3D pos1 = m_impl->nodes[m_impl->edges[edgeId].from()].pos();
        Coord3D pos2 = m_impl->nodes[m_impl->edges[edgeId].to()].pos();
        Coord3D center = (pos1 + pos2) / 2.0;
        m_impl->handles[edgeId].pos() = center;
        m_impl->transitionLabelnodes[edgeId].pos() = center;
      }

      // creates, and increases count for node
      Node& increaseNode(size_t nodeId)
      {
        if (m_nodes.count(nodeId))
        {
          Node& node = m_nodes[nodeId];
          ++node.count;
          return node;
        }

        Node& node = m_nodes[nodeId];
        node.id = nodeId;
        node.index = m_nodeIndices.size();
        m_nodeIndices.push_back(nodeId);

        for (size_t i = 0; i < m_impl->edges.size(); ++i)
        {
          ::Graph::Edge& edge = m_impl->edges[i];
          if (edge.from() == nodeId)
            node.outEdges.push_back(i);
          if (edge.to() == nodeId)
            node.inEdges.push_back(i);
        }

        node.count = 1;

        repositionNode(node);
        return node;
      }

      // creates, and increases count for edge
      Edge& increaseEdge(size_t edgeId)
      {
        if (m_edges.count(edgeId))
        {
          Edge& edge = m_edges[edgeId];
          ++edge.count;
          return edge;
        }
        
        Edge& edge = m_edges[edgeId];
        edge.id = edgeId;
        edge.index = m_edgeIndices.size();
        m_edgeIndices.push_back(edgeId);

        edge.count = 1;

        repositionEdge(edgeId);
        return edge;
      }

      // decreases selection count for node, and purges
      void decreaseNode(size_t nodeId)
      {
        if (!m_nodes.count(nodeId))
          return;

        Node& node = m_nodes[nodeId];
        if (--node.count < 1)
        {
          size_t last = m_nodeIndices.size() - 1;
          if (node.index < last)
          {
            size_t lastId = m_nodeIndices[last];
            m_nodeIndices[node.index] = lastId;
            m_nodes[lastId].index = node.index;
          }
          m_nodeIndices.pop_back();
          m_nodes.erase(nodeId);
        }
      }

      // decreases selection count for edge, and purges
      void decreaseEdge(size_t edgeId)
      {
        if (!m_edges.count(edgeId))
          return;

        Edge& edge = m_edges[edgeId];
        if (--edge.count < 1)
        {
          size_t last = m_edgeIndices.size() - 1;
          if (edge.index < last)
          {
            size_t lastId = m_edgeIndices[last];
            m_edgeIndices[edge.index] = lastId;
            m_edges[lastId].index = edge.index;
          }
          m_edgeIndices.pop_back();
          m_edges.erase(edgeId);
        }
      }

      void updateBridges()
      {
        // Todo: implement
      }

    public:
      Selection(detail::GraphImplBase* impl)
        : nodes(m_nodeIndices), edges(m_edgeIndices), m_impl(impl) {}

      // expand outgoing transitions and states for specified node
      void expand(size_t nodeId)
      {
        Node& node = increaseNode(nodeId);
        for (size_t i = 0; i < node.outEdges.size(); ++i)
        {
          size_t edgeId = node.outEdges[i];
          increaseNode(m_impl->edges[edgeId].to());
          increaseEdge(edgeId);
        }
        updateBridges();
      }

      // contract outgoing transitions and states for specified node
      void contract(size_t nodeId)
      {
        if (m_nodes.count(nodeId) != 0)
        {
          Node& node = m_nodes[nodeId];
          for (size_t i = 0; i < node.outEdges.size(); ++i)
          {
            size_t edgeId = node.outEdges[i];
            decreaseEdge(edgeId);
            decreaseNode(m_impl->edges[edgeId].to());
          }
        }
        decreaseNode(nodeId);
        updateBridges();
      }

      // tells whether a node when contracted would leave unconnected components
      bool isBridge(size_t nodeId)
      {
        if (!m_nodes.count(nodeId))
          return false;

        // If one of the out-edges is a bridge and would unselect: true
        /*Node &node = m_nodes[nodeId];
        for (size_t i = 0; i < node.outEdges.size(); ++i)
        {
          size_t edgeId = node.outEdges[i];
          if (m_edges.count(edgeId) != 0)
          {
            Edge &edge = m_edges[edgeId];
            if (edge.bridge && edge.count <= 1)
              return true;
          }
        }*/

        return false;
      }
  };

  Graph::Graph() : m_sel(nullptr), m_lock(QReadWriteLock::Recursive)
  {
    m_type = mcrl2::lts::lts_lts;
    m_impl = new detail::GraphImpl<mcrl2::lts::probabilistic_lts_lts_t>;
    m_empty = QString("");
  }

  Graph::~Graph()
  {
    delete m_impl;
    if (m_sel != nullptr)
    {
      delete m_sel;
      m_sel = nullptr;
    }
  }

  size_t Graph::edgeCount() const
  {
    return m_impl->edges.size();
  }

  size_t Graph::nodeCount() const
  {
    return m_impl->nodes.size();
  }

  size_t Graph::transitionLabelCount() const
  {
    return m_impl->transitionLabels.size();
  }

  size_t Graph::stateLabelCount() const
  {
    return m_impl->stateLabels.size();
  }

  size_t Graph::initialState() const
  {
    return m_impl->initialState;
  }

  bool Graph::isTau(size_t labelindex) const
  {
    return m_impl->is_tau(labelindex);
  }

  void Graph::createImpl(mcrl2::lts::lts_type itype)
  {
    switch (itype)
    {
      case mcrl2::lts::lts_aut:
        m_type = mcrl2::lts::lts_aut;
        m_impl = new detail::GraphImpl<mcrl2::lts::probabilistic_lts_aut_t>;
        // m_impl = new detail::GraphImpl<mcrl2::lts::lts_aut_t>;
        break;
      case mcrl2::lts::lts_dot:
        throw mcrl2::runtime_error("Cannot read a .dot file anymore.");
      case mcrl2::lts::lts_fsm:
        m_type = mcrl2::lts::lts_fsm;
        m_impl = new detail::GraphImpl<mcrl2::lts::probabilistic_lts_fsm_t>;
        // m_impl = new detail::GraphImpl<mcrl2::lts::lts_fsm_t>;
        break;
      case mcrl2::lts::lts_lts:
      default:
        m_type = mcrl2::lts::lts_lts;
        m_impl = new detail::GraphImpl<mcrl2::lts::probabilistic_lts_lts_t>;
        // m_impl = new detail::GraphImpl<mcrl2::lts::lts_lts_t>;
        break;
    }
  }

  void Graph::load(const QString &filename, const Coord3D& min, const Coord3D& max)
  {
    m_lock.lockForWrite();

    mcrl2::lts::lts_type guess = mcrl2::lts::detail::guess_format(filename.toUtf8().constData());
    delete m_impl;
    createImpl(guess);
    m_impl->load(filename, min, max);
    if (m_sel != nullptr)
    {
      delete m_sel;
      m_sel = nullptr;
    }

    m_lock.unlock();
  }

  void Graph::loadXML(const QString& filename)
  {
    m_lock.lockForWrite();

    QDomDocument xml;
    QFile file(filename);
    if(!file.open( QFile::ReadOnly ))
    {
      mCRL2log(mcrl2::log::error) << "Could not open XML file: " << filename.toStdString() << std::endl;
      return;
    }
    QString errorMsg;
    if(!xml.setContent(&file, false, &errorMsg))
    {
      file.close();
      mCRL2log(mcrl2::log::error) << "Could not parse XML file: " << errorMsg.toStdString() << std::endl;
      return;
    }
    file.close();

    QDomElement root = xml.documentElement();
    if(root.tagName() != "Graph")
    {
      mCRL2log(mcrl2::log::error) << "XML contains no valid graph" << std::endl;
      return;
    }

    delete m_impl;
    mcrl2::lts::lts_type itype = (mcrl2::lts::lts_type) root.attribute("type").toInt();
    createImpl(itype);

    m_impl->nodes.resize(root.attribute("states").toInt());
    m_impl->edges.resize(root.attribute("transitions").toInt());
    m_impl->handles.resize(root.attribute("transitions").toInt());

    m_impl->transitionLabels.resize(root.attribute("transitionlabels").toInt());
    m_impl->transitionLabelnodes.resize(root.attribute("transitions").toInt());

    m_impl->stateLabels.resize(root.attribute("statelabels").toInt());
    m_impl->stateLabelnodes.resize(root.attribute("states").toInt());

    QDomNode node = root.firstChild();
    while (!node.isNull()) 
    {
      QDomElement e = node.toElement();

      if (e.tagName() == "StateLabel") {
        m_impl->stateLabels[e.attribute("value").toInt()] = e.attribute("label");
      }
      if (e.tagName() == "State") 
      {
        m_impl->nodes[e.attribute("value").toInt()]=
           NodeNode(
              Coord3D(e.attribute("x").toFloat(), e.attribute("y").toFloat(), e.attribute("z").toFloat()),
              e.attribute("locked").toInt(),   // anchored is equal to locked.
              e.attribute("locked").toInt(),
              0.0f,                            // selected
              e.attribute("red").toFloat(),
              e.attribute("green").toFloat(),
              e.attribute("blue").toFloat(),
              e.attribute("is_probabilistic").toInt());

        if (e.attribute("isInitial").toInt())
        {  
          m_impl->initialState = e.attribute("value").toInt();
        }
      }
      if (e.tagName() == "StateLabelNode") 
      {
        m_impl->stateLabelnodes[e.attribute("value").toInt()]=
          LabelNode(
              Coord3D(e.attribute("x").toFloat(),e.attribute("y").toFloat(),e.attribute("z").toFloat()),
              e.attribute("locked").toInt(),   // anchored is equal to locked.
              e.attribute("locked").toInt(),
              0.0f,                            // selected
              e.attribute("red").toFloat(),
              e.attribute("green").toFloat(),
              e.attribute("blue").toFloat(),
              e.attribute("labelindex").toInt());

      }

      if (e.tagName() == "TransitionLabel") 
      {
        m_impl->transitionLabels[e.attribute("value").toInt()]=LabelString(e.attribute("isTau").toInt(),e.attribute("label"));
      }
      if (e.tagName() == "Transition") 
      {
        m_impl->edges[e.attribute("value").toInt()]=Edge(e.attribute("from").toInt(),e.attribute("to").toInt());
        m_impl->handles[e.attribute("value").toInt()]=
          Node(Coord3D(e.attribute("x").toFloat(),e.attribute("y").toFloat(),e.attribute("z").toFloat()),
               e.attribute("locked").toInt(),  // anchored is equal to locked.
               e.attribute("locked").toInt(),
               0.0f);                          // selected
      }
      if (e.tagName() == "TransitionLabelNode") 
      {
        m_impl->transitionLabelnodes[e.attribute("value").toInt()]=
          LabelNode(
              Coord3D(e.attribute("x").toFloat(),e.attribute("y").toFloat(),e.attribute("z").toFloat()),
              e.attribute("locked").toInt(),   // anchored is equal to locked.
              e.attribute("locked").toInt(),
              0.0f,                            // selected
              e.attribute("red").toFloat(),
              e.attribute("green").toFloat(),
              e.attribute("blue").toFloat(),
              e.attribute("labelindex").toInt());
      }

      node = node.nextSibling();
    }

    if (m_sel != nullptr)
    {
      delete m_sel;
      m_sel = nullptr;
    }

    m_lock.unlock();
  }

  void Graph::saveXML(const QString& filename)
  {
    m_lock.lockForRead();

    QDomDocument xml;
    QDomElement root = xml.createElement("Graph");
    root.setAttribute("type", (int)m_type);
    root.setAttribute("states", (int)nodeCount());
    root.setAttribute("transitions", (int)edgeCount());
    root.setAttribute("statelabels", (int)stateLabelCount());
    root.setAttribute("transitionlabels", (int)transitionLabelCount());
    xml.appendChild(root);

    for (size_t i = 0; i < stateLabelCount(); ++i)
    {
      QDomElement stateL = xml.createElement("StateLabel");
      stateL.setAttribute("value", (int)i);
      stateL.setAttribute("label", stateLabelstring(i));
      root.appendChild(stateL);
    }

    for (size_t i = 0; i < nodeCount(); ++i)
    {
      QDomElement state = xml.createElement("State");
      state.setAttribute("value", (int)i);
      state.setAttribute("x", node(i).pos().x);
      state.setAttribute("y", node(i).pos().y);
      state.setAttribute("z", node(i).pos().z);
      state.setAttribute("locked", node(i).locked());
      state.setAttribute("isInitial", (int)(i == initialState()));
      state.setAttribute("red", node(i).color(0));
      state.setAttribute("green", node(i).color(1));
      state.setAttribute("blue", node(i).color(2));
      state.setAttribute("is_probabilistic", node(i).is_probabilistic());
      root.appendChild(state);

      QDomElement stateL = xml.createElement("StateLabelNode");
      stateL.setAttribute("value", (int)i);
      stateL.setAttribute("labelindex", (int)stateLabel(i).labelindex());
      stateL.setAttribute("x", stateLabel(i).pos().x);
      stateL.setAttribute("y", stateLabel(i).pos().y);
      stateL.setAttribute("z", stateLabel(i).pos().z);
      stateL.setAttribute("locked", stateLabel(i).locked());
      stateL.setAttribute("red", stateLabel(i).color(0));
      stateL.setAttribute("green", stateLabel(i).color(1));
      stateL.setAttribute("blue", stateLabel(i).color(2));
      root.appendChild(stateL);
    }

    for (size_t i = 0; i < transitionLabelCount(); ++i)
    {
      QDomElement edgL = xml.createElement("TransitionLabel");
      edgL.setAttribute("value", (int)i);
      edgL.setAttribute("label", transitionLabelstring(i));
      root.appendChild(edgL);
    }

    for (size_t i = 0; i < edgeCount(); ++i)
    {
      QDomElement edg = xml.createElement("Transition");
      edg.setAttribute("value", (int)i);
      edg.setAttribute("from", (int)edge(i).from());
      edg.setAttribute("to", (int)edge(i).to());
      edg.setAttribute("x", handle(i).pos().x);
      edg.setAttribute("y", handle(i).pos().y);
      edg.setAttribute("z", handle(i).pos().z);
      edg.setAttribute("locked", handle(i).locked());
      root.appendChild(edg);

      QDomElement edgL = xml.createElement("TransitionLabelNode");
      edgL.setAttribute("value", (int)i);
      edgL.setAttribute("labelindex", (int)transitionLabel(i).labelindex());
      edgL.setAttribute("x", transitionLabel(i).pos().x);
      edgL.setAttribute("y", transitionLabel(i).pos().y);
      edgL.setAttribute("z", transitionLabel(i).pos().z);
      edgL.setAttribute("locked", transitionLabel(i).locked());
      edgL.setAttribute("red", transitionLabel(i).color(0));
      edgL.setAttribute("green", transitionLabel(i).color(1));
      edgL.setAttribute("blue", transitionLabel(i).color(2));
      root.appendChild(edgL);
    }

    QFile data(filename);
    if (data.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream out(&data);
        xml.save(out, 2);
    }

    // Todo: Perhaps save selection too

    m_lock.unlock();
  }

  Edge Graph::edge(size_t index) const
  {
    return m_impl->edges[index];
  }

  NodeNode& Graph::node(size_t index) const
  {
    return m_impl->nodes[index];
  }

  Node& Graph::handle(size_t edge) const
  {
    return m_impl->handles[edge];
  }

  LabelNode& Graph::transitionLabel(size_t edge) const
  {
    return m_impl->transitionLabelnodes[edge];
  }

  LabelNode& Graph::stateLabel(size_t edge) const
  {
    return m_impl->stateLabelnodes[edge];
  }

  const QString& Graph::transitionLabelstring(size_t labelindex) const
  {
    if (labelindex >= m_impl->transitionLabels.size())
      return m_empty;
    return m_impl->transitionLabels[labelindex].label();
  }

  const QString& Graph::stateLabelstring(size_t labelindex) const
  {
    if (labelindex >= m_impl->stateLabels.size())
      return m_empty;
    return m_impl->stateLabels[labelindex];
  }

  void Graph::clip(const Coord3D& min, const Coord3D& max)
  {
    for (std::vector<NodeNode>::iterator it = m_impl->nodes.begin(); it != m_impl->nodes.end(); ++it)
      it->pos().clip(min, max);
    for (std::vector<LabelNode>::iterator it = m_impl->transitionLabelnodes.begin(); it != m_impl->transitionLabelnodes.end(); ++it)
      it->pos().clip(min, max);
    for (std::vector<Node>::iterator it = m_impl->handles.begin(); it != m_impl->handles.end(); ++it)
      it->pos().clip(min, max);
  }

  void Graph::lock()
  {
    m_lock.lockForRead();
  }

  void Graph::unlock()
  {
    m_lock.unlock();
  }

  void Graph::makeSelection()
  {
    m_lock.lockForWrite();

    if (m_sel != nullptr)
    {
      delete m_sel;
    }
    m_sel = new Selection(m_impl);

    m_lock.unlock();
  }

  void Graph::discardSelection()
  {
    m_lock.lockForWrite();

    if (m_sel != nullptr)
    {
      delete m_sel;
      m_sel = nullptr;
    }

    // Deactive all nodes
    for (std::vector<NodeNode>::iterator it = m_impl->nodes.begin(); it != m_impl->nodes.end(); ++it)
      it->m_active = false;

    m_lock.unlock();
  }

  void Graph::toggleActive(size_t index)
  {
    m_lock.lockForWrite(); // enter critical section

    if (m_sel != nullptr && index < m_impl->nodes.size())
    {
      NodeNode& node = m_impl->nodes[index];
      if (node.m_active)
      {
        m_sel->contract(index);
      }
      else
      {
        m_sel->expand(index);
      }
      node.m_active = !node.m_active;
    }

    m_lock.unlock(); // exit critical section
  }

  bool Graph::isToggleable(size_t index)
  {
    if (m_sel == nullptr || index >= m_impl->nodes.size())
      return false;

    m_lock.lockForRead();

    // active node count:
    size_t count = 0;
    for (size_t i = 0; i < m_sel->nodes.size(); ++i)
      if (m_impl->nodes[m_sel->nodes[i]].m_active)
        ++count;
    
    NodeNode& node = m_impl->nodes[index];
    bool toggleable = !node.m_active || (!m_sel->isBridge(index) && count > 1);

    m_lock.unlock();

    return toggleable;
  }

  bool Graph::isBridge(size_t index) const
  {
    return m_sel->isBridge(index);
  }

  bool Graph::hasSelection() const
  {
    return m_sel != nullptr;
  }

  size_t Graph::selectionEdge(size_t index) const
  {
    return m_sel->edges[index];
  }

  size_t Graph::selectionNode(size_t index) const
  {
    return m_sel->nodes[index];
  }

  size_t Graph::selectionEdgeCount() const
  {
    if (m_sel == nullptr)
      return 0;
    return m_sel->edges.size();
  }

  size_t Graph::selectionNodeCount() const
  {
    if (m_sel == nullptr)
      return 0;
    return m_sel->nodes.size();
  }
  
}
