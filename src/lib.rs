use ospf_parser::{
    Ospfv3LinkStateAdvertisementHeader, 
    Ospfv3LinkStateType,
    Ospfv3LinkStateAdvertisement,
    Ospfv3RouterLink,
    Ospfv3IPv6AddressPrefix,
};

use std::{
    mem::transmute,
    fs::{File},
    io::prelude::*,
    collections::HashMap,
    net::Ipv6Addr
};

use serde::{Deserialize, Serialize};
// use serde_json::Result;

use nom_derive::Parse;
use std::ffi::{c_void};

mod graph;
use crate::graph::{DAG, WeightedHashSuccessors, HashSuccessors, DAGOrder};

#[repr(C)]
#[derive(Debug, Clone)]
#[derive(Serialize, Deserialize)]
pub struct TimeInfo {
    tv_sec: i64,
    tv_nsec: i64,
}

#[repr(C)]
#[derive(Debug, Clone)]
#[derive(Serialize, Deserialize)]
pub struct NodeInfo {
    src: u32,
    dst: u32,
    depth: u32
}

/// Structure embedding oFIB timings for each remote peer connection
#[repr(C)]
#[derive(Debug, Clone)]
#[derive(Serialize, Deserialize)]
pub struct RemoteInfo {
    co_init: TimeInfo,
    co_created: TimeInfo,
    stream_init: TimeInfo,
    stream_created: TimeInfo,
    lsa_sent: TimeInfo,
    conf_recep: TimeInfo, 
    node_info: NodeInfo
}

/// Test doc
#[derive(Debug)]
pub struct Ofib {
    //lsas: Vec<Ospfv3LinkStateAdvertisementHeader>,
    successors: WeightedHashSuccessors<usize, u16>,
    prefixes: HashMap<u32, Vec<Ipv6Addr>>,
    rids: Vec<u32>,
    file: File,
    infos: Vec<RemoteInfo>,
}

fn invert_endianess(buf: &[u8], len: usize) -> Vec<u8> {
    let mut ret: Vec<u8> = Vec::new();
    for i in 0..len / 4 {
        ret.extend_from_slice(&buf[i*4 .. (i*4)+4].iter().rev().map(|&i| i).collect::<Vec<u8>>());
    }
    ret
}

#[repr(C)]
pub struct Buffer {
    buffer: *const c_void,
    len: u64
}



#[macro_export]
macro_rules! get_lsa_body {
    ( $hdr:expr, $body:expr ) => {
        {
            let body_size: usize = ($hdr.length - 20).into();
            //let body_buf_raw: &[u8] = unsafe {std::slice::from_raw_parts($body as *const _, body_size)};
            invert_endianess($body, body_size)
        }
    };
}

impl Ofib {

    pub fn new(rid: u32) -> Self {
        Ofib {
            successors: WeightedHashSuccessors::new(),
            prefixes: HashMap::new(),
            file: File::create(format!("/tmp/node{}.ofib", rid)).unwrap(),
            rids: Vec::new(),
            infos: Vec::new(),
        }
    }

    /// Parse OSPF LSAs.
    ///
    /// This functions looks for two kinds of LSAs:
    /// 
    /// (1) RouterLSA: It contains the neighborhood of a given router. Collecting them allows to
    /// rebuild the topology graph.
    ///
    /// (2) IntraAreaPrefix: It contains the prefixes advertised by a given router. We are
    /// interested in loopback addresses. They are indicated by the following option pattern in the
    /// LSAs options:
    /// referenced_ls_type: 0x2001,
    /// referenced_link_state_id: 0x0,
    /// referenced_advertising_router: <advertising_rid>,
    ///
    /// See https://www.rfc-editor.org/rfc/rfc5340#section-4.4.3.9
    ///
    pub fn add_lsa(&mut self, hdr: &Ospfv3LinkStateAdvertisementHeader, body: &[u8]) {
        // std::mem::size_of::<Ospfv3LinkStateAdvertisementHeader>
        //println!("add_lsa {:02x?}", body);

        let body_size: usize = (hdr.length - 20).into();

        match hdr.link_state_type {
            Ospfv3LinkStateType::RouterLSA => {
                // Collect topology

                let body_buf = get_lsa_body!(hdr, body);
                //println!("");

                // let _flags: u8 = body_buf[0] as u8;

                // Convert 3-byte wide uint32 in 4-byte wide
                /*let mut dst = [0; 4];
                let (_, right) = dst.split_at_mut(1);
                right.clone_from_slice(&body_buf[1..4]);
                let _options: u32 = u32::from_be_bytes(dst.try_into().unwrap()); */

                //let _written = self.file.write(format!("flags <{:#x}> options <{:#x}>\n", _flags, _options).as_bytes());

                // Get RouterLink list
                let mut start = 4;
                for _i in 0 .. (body_size - 4) / 16 {
                    let a: Ospfv3RouterLink = Ospfv3RouterLink::parse(&body_buf[start .. start+16]).unwrap().1;
                    //let _written = self.file.write(format!("{:#x?}\n", &a).as_bytes());
                    let pair = (a.neighbor_router_id as usize, a.metric);
                    match self.successors.get_mut(&(hdr.advertising_router as usize)) {
                        None => {
                            self.successors.insert(hdr.advertising_router as usize, vec![pair]);
                        },
                        Some(entry) => {
                            entry.push(pair);
                        }
                    }
                    start += 16;
                }
                //let _written = self.file.write(format!("{:#x?}\n", self.successors).as_bytes());
            },
            Ospfv3LinkStateType::IntraAreaPrefixLSA => {
                // Collect mapping router id: prefix
                let body_buf = get_lsa_body!(hdr, body);
                //println!("body_buf {:02x?}\n{:02x?}", body_buf, &body_buf[0..2]);
                let n_prefixes: u16 = u16::from_be_bytes(body_buf[0..2].try_into().unwrap());
                let referenced_ls_type: u16 = u16::from_be_bytes(body_buf[2..4].try_into().unwrap());
                let referenced_link_state_id: u32 = u32::from_be_bytes(body_buf[4..8].try_into().unwrap());
                let referenced_advertising_router: u32 = u32::from_be_bytes(body_buf[8..12].try_into().unwrap());
                //println!("{:02x?} {:02x?}", n_prefixes, referenced_ls_type);

                if 
                    Ospfv3LinkStateType(referenced_ls_type) != Ospfv3LinkStateType::RouterLSA || 
                    referenced_link_state_id != 0 || 
                    referenced_advertising_router != hdr.advertising_router 
                { return; }

                // Get router prefixes
                let mut start = 12;
                for _ in 0..n_prefixes {
                    let prefix_hdr = &body_buf[start..start+4];
                    // Prefix length is the number of bits contained in the filed address_prefix
                    let prefix_len = prefix_hdr[0] / 8;
                    let v3_prefix_size = prefix_len as usize + 4;
                    ////println!("{:02x?} {:02x?}", prefix_hdr, &body_buf[start..]);
                    let prefix = Ospfv3IPv6AddressPrefix::parse(&body_buf[start..start+v3_prefix_size]).unwrap().1;
                    let a: [u8; 16] = prefix.address_prefix.try_into().unwrap();
                    let final_prefix = std::net::Ipv6Addr::from(a);
                    self.prefixes.entry(hdr.advertising_router).and_modify(|v| v.push(final_prefix)).or_insert(vec![final_prefix]);

                    //let _written = self.file.write(format!("{:#x?}\n", prefix).as_bytes());
                    start += v3_prefix_size;
                }
            },
            _ => {}
        }

    }

    fn converge(&mut self, neighbor: u32, rid: u32, order_type: u8) -> Vec<Ipv6Addr> {
        DAG::order_failure(&self.successors, &(neighbor as usize), &(rid as usize), 65534, order_type)
        .iter()
        .for_each(|rid| self.rids.push(*rid as u32));
        
        self.rids
        .iter()
        .map(|rid| *self.prefixes.get(&(*rid as u32)).unwrap().first().unwrap())
        .collect()
    }

    fn register_timers(&mut self, peer: RemoteInfo) {
        self.infos.push(peer.clone());
    }

    fn dump_timers(&mut self) {
        match serde_json::to_string(&self.infos) {
            Ok(content) => {
                let _written = self.file.write(content.as_bytes());
            },
            Err(_) => {}
        }
    }

    /* ####################### FFI ######################### */

    /// Test doc
    #[no_mangle]
    pub extern "C" fn ofib_init(rid: u32) -> *mut Self {
        Box::into_raw(Box::new(Self::new(rid)))
    }

    /// Parse OSPF LSAs.
    ///
    /// This functions looks for two kinds of LSAs:
    /// 
    /// (1) RouterLSA: It contains the neighborhood of a given router. Collecting them allows to
    /// rebuild the topology graph.
    ///
    /// (2) IntraAreaPrefix: It contains the prefixes advertised by a given router. We are
    /// interested in loopback addresses. They are indicated by the following option pattern in the
    /// LSAs options:
    /// referenced_ls_type: 0x2001,
    /// referenced_link_state_id: 0x0,
    /// referenced_advertising_router: <advertising_rid>,
    ///
    /// See https://www.rfc-editor.org/rfc/rfc5340#section-4.4.3.9
    ///
    #[no_mangle]
    pub extern "C" fn ofib_add_lsa(&mut self, hdr: *const Ospfv3LinkStateAdvertisementHeader, body: *const c_void) {

        let hdr_ptr = unsafe {&*hdr};
        let body_size: usize = (hdr_ptr.length - std::mem::size_of::<Ospfv3LinkStateAdvertisementHeader>() as u16) as usize;
        let body_buf_raw: &[u8] = unsafe {std::slice::from_raw_parts(body as *const _, body_size)};
        self.add_lsa(hdr_ptr, body_buf_raw);

        /*let hdr_ptr = unsafe {&*hdr};
        // std::mem::size_of::<Ospfv3LinkStateAdvertisementHeader>

        let body_size: usize = (hdr_ptr.length - 20).into();

        // Parse only Router LSAs
        match hdr_ptr.link_state_type {
            Ospfv3LinkStateType::RouterLSA => {

                //let body_buf_raw: &[u8] = unsafe {std::slice::from_raw_parts(body as *const _, body_size)};

                //let body_buf_parsed = invert_endianess(body_buf_raw, body_size);
                //let body_buf = body_buf_parsed.as_slice();
                let body_buf = get_lsa_body!(hdr_ptr, body);

                let _flags: u8 = body_buf[0] as u8;

                // Convert 3-byte wide uint32 in 4-byte wide
                let mut dst = [0; 4];
                let (_, right) = dst.split_at_mut(1);
                right.clone_from_slice(&body_buf[1..4]);
                let _options: u32 = u32::from_be_bytes(dst.try_into().unwrap()); 

                let _written = self.file.write(format!("flags <{:#x}> options <{:#x}>\n", _flags, _options).as_bytes());

                // Get RouterLink list
                let mut start = 4;
                for _i in 0 .. (body_size - 4) / 16 {
                    let a: Ospfv3RouterLink = Ospfv3RouterLink::parse(&body_buf[start .. start+16]).unwrap().1;
                    let _written = self.file.write(format!("{:#x?}\n", &a).as_bytes());
                    let pair = (a.neighbor_router_id as usize, a.metric);
                    match self.successors.get_mut(&(hdr_ptr.advertising_router as usize)) {
                        None => {
                            self.successors.insert(hdr_ptr.advertising_router as usize, vec![pair]);
                        },
                        Some(entry) => {
                            entry.push(pair);
                        }
                    }
                    start += 16;
                }
                let _written = self.file.write(format!("{:#x?}\n", self.successors).as_bytes());
            },
            Ospfv3LinkStateType::IntraAreaPrefixLSA => { 
                let body_buf = get_lsa_body!(hdr_ptr, body);

                let n_prefixes: u16 = u16::from_be_bytes(body_buf[0..1].try_into().unwrap());
                let referenced_ls_type: u16 = u16::from_be_bytes(body_buf[2..3].try_into().unwrap());
                let referenced_link_state_id: u32 = u32::from_be_bytes(body_buf[4..7].try_into().unwrap());
                let referenced_advertising_router: u32 = u32::from_be_bytes(body_buf[8..11].try_into().unwrap());

                if 
                    Ospfv3LinkStateType(referenced_ls_type) != Ospfv3LinkStateType::RouterLSA || 
                    referenced_link_state_id != 0 || 
                    referenced_advertising_router != hdr_ptr.advertising_router 
                { return; }

                // Get router prefixes
                let mut start = 12;
                for _ in 0..n_prefixes {
                    let prefix = Ospfv3IPv6AddressPrefix::parse(&body_buf[start..start+192]);
                    let _written = self.file.write(format!("{:#x?}\n", prefix).as_bytes());
                    start += 192;
                }
            },
            _ => {}
        }*/
    }

    #[no_mangle]
    pub extern "C" fn ofib_converge(&mut self, neighbor: u32, rid: u32, order_type: u8) -> Buffer {
        let prefixes: Vec<[u8; 16]> = self.converge(neighbor, rid, order_type)
        .iter().map(|prefix| prefix.octets()).collect();

        let ptr = prefixes.as_slice().as_ptr() as *const _;
        let len = prefixes.len();

        // TODO: possible memory leak to fix
        std::mem::forget(prefixes);
        Buffer {
            buffer: ptr,
            len: len as u64
        }
    }

    #[no_mangle]
    pub extern "C" fn ofib_free(&mut self) {
        let _drop: Box<Self> = unsafe { transmute(self) };
    }

    #[no_mangle]
    pub extern "C" fn ofib_get_rids(&self) -> Buffer {
        let rids = self.rids.clone();
        let ptr = rids.as_slice().as_ptr() as *const _;
        let len = rids.len();
        std::mem::forget(rids);
        Buffer {
            buffer: ptr,
            len: len as u64
        }
    }

    #[no_mangle]
    pub extern "C" fn ofib_register_timers(&mut self, peer: RemoteInfo) {
        self.register_timers(peer);
    }

    #[no_mangle]
    pub extern "C" fn ofib_dump_timers(&mut self) {
        self.dump_timers();
    }

    #[no_mangle]
    pub extern "C" fn ofib_get_registerd_timers(&self) -> u32 {
        self.infos.len() as u32
    }
}

#[no_mangle]
pub extern "C" fn parse_lsa(lsa_hdr: *const Ospfv3LinkStateAdvertisementHeader, _lsa_body: *const Ospfv3LinkStateAdvertisement) {
    let lsa_hdr_ptr = unsafe { &*lsa_hdr };
    //println!("{:#?} {}", lsa_hdr_ptr, lsa_hdr_ptr.link_state_type == Ospfv3LinkStateType::RouterLSA);
}

#[cfg(test)]
mod timers {
    use crate::{Ofib, NodeInfo, TimeInfo, RemoteInfo};

    #[test]
    fn timers_dump_test() {
        let mut ofib = Ofib::new(0);

        for i in 0..10 {
            let timer = RemoteInfo {
                co_init: TimeInfo {
                    tv_sec: 0,
                    tv_nsec: 0
                },
                co_created: TimeInfo {
                    tv_sec: 0,
                    tv_nsec: 0
                },
                stream_init: TimeInfo {
                    tv_sec: 0,
                    tv_nsec: 0
                },
                stream_created: TimeInfo {
                    tv_sec: 0,
                    tv_nsec: 0
                },
                lsa_sent: TimeInfo {
                    tv_sec: 0,
                    tv_nsec: 0
                },
                conf_recep: TimeInfo {
                    tv_sec: 0,
                    tv_nsec: 0
                },
                node_info: NodeInfo {
                    src: 18,
                    dst: 19,
                    depth: 4
                }
            };

            ofib.register_timers(timer);
        }
        //println!("{:#?}", ofib.infos);
        ofib.dump_timers();
    }
}

#[cfg(test)]
mod house {

    use std::{
        mem::size_of,
        collections::HashMap,
        net::Ipv6Addr
    };

    use std::fs::read;
    use crate::{Ofib, invert_endianess, WeightedHashSuccessors};
    use nom_derive::Parse;
    use ospf_parser::{
        Ospfv3LinkStateAdvertisementHeader,
        Ospfv3Packet,
        parse_ospfv3_packet,
        Ospfv3PacketHeader
    };

    fn load_lsp() -> Vec<(Ospfv3LinkStateAdvertisementHeader, Vec<u8>)> {
        let mut ret = Vec::new();
        for entry in std::fs::read_dir("tests/house_lsps/").unwrap() {
            let entry = entry.unwrap();
            let path = entry.path();
            if ! path.is_dir() {
                let path = path.to_str().unwrap();
                let content = read(path).unwrap();
                if let Some(split) = split_lsp(&content) {
                    //println!("load_lsp {:#02x?}", split);
                    for (hdr, body) in split {
                        let body = invert_endianess(body, hdr.length as usize - size_of::<Ospfv3LinkStateAdvertisementHeader>());
                        ret.push((hdr, body.to_vec()));
                    }
                }

            }
        }
        ret
    }

    fn split_lsp(lsp: &[u8]) -> Option<Vec<(Ospfv3LinkStateAdvertisementHeader, &[u8])>> {
        let raw_pkt = parse_ospfv3_packet(lsp).unwrap().1;


        if let Ospfv3Packet::LinkStateUpdate(pkt) = raw_pkt {
            ////println!("{:#x?}", pkt);

            // LSA header length = LSA header + #LSA (32bits)
            let skip_pkt_header = size_of::<Ospfv3PacketHeader>() + size_of::<u32>();
            let lsp_body = &lsp[skip_pkt_header..];
            ////println!("{:02x?}", lsp_body);

            let mut ret = Vec::new();

            let mut skip = 0;
            let header_len = size_of::<Ospfv3LinkStateAdvertisementHeader>();

            for _lsa in pkt.lsa {
                let length_offset = skip + header_len - size_of::<u16>();
                let lsa_length = u16::from_be_bytes(lsp_body[length_offset..length_offset+size_of::<u16>()].try_into().unwrap());
                let lsa_hdr = Ospfv3LinkStateAdvertisementHeader::parse(&lsp_body[skip..skip+header_len]).unwrap().1;
                let lsa_body_offset = skip + header_len;
                let lsa_body = &lsp_body[lsa_body_offset..lsa_body_offset + (lsa_length as usize - header_len)];
                ////println!("{:02x?}", lsa_body);
                ret.push((lsa_hdr, lsa_body));
                skip += lsa_length as usize;
            }
            return Some(ret);
        }
        None
    }

    fn gen_rid_prefix_mapping() -> HashMap<u32, Vec<Ipv6Addr>> {
        let mut base = [0xfc, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let mut ret: HashMap<u32, Vec<Ipv6Addr>> = HashMap::new();
        for i in 1..7 {
            base[6] = i;
            let prefix = Ipv6Addr::from(base);
            //println!("{:x?}", prefix);
            ret.insert(i as u32, vec![prefix]);
        }
        ret
    }

    #[test]
    fn inject_lsas() {

        let mut ofib = Ofib::new(0);
        load_lsp().iter().for_each(|(hdr, body)| ofib.add_lsa(&hdr, &body.as_slice()));

        //println!("{:#02x?}", &ofib);

        let mapping = gen_rid_prefix_mapping();
        //println!("{:#?}", mapping);

        // Test collected prefixes mapping
        for (rid, prefix_list) in &mapping {
            assert!(ofib.prefixes.contains_key(&rid));
            assert_eq!(prefix_list, mapping.get(&rid).unwrap());
        }

        for (rid, prefix_list) in &ofib.prefixes {
            assert!(mapping.contains_key(&rid));
            assert_eq!(prefix_list, ofib.prefixes.get(&rid).unwrap());
        }

        // TODO: test depths vector
    }

    // TODO: remove duplicate with graph.rs tests
    fn gen_house() -> WeightedHashSuccessors<usize, u16> {
        let mut topo: WeightedHashSuccessors<usize, u16> = WeightedHashSuccessors::new();
        topo.insert(1, vec![(2, 1), (3, 10)]);
        topo.insert(2, vec![(1, 1), (3, 1), (5, 10), (4, 1)]);
        topo.insert(3, vec![(1, 10), (2, 1), (5, 1), (6, 1)]);
        topo.insert(4, vec![(2, 1), (6, 1)]);
        topo.insert(5, vec![(2, 10), (3, 1)]);
        topo.insert(6, vec![(4, 1), (3, 1)]);
        topo
    }

//    #[test]
    fn successors() {
        let mut ofib = Ofib::new(0);
        load_lsp().iter().for_each(|(hdr, body)| ofib.add_lsa(&hdr, &body.as_slice()));

        for (node, mut successors) in gen_house() {
            //println!("{}", node);
            let mut got = ofib.successors.get_mut(&node).unwrap();
            successors.sort();
            got.sort();
            assert_eq!(successors, *got);
        }
    }


    //#[test]
    fn order_on_failures() {
        let mut ofib = Ofib::new(0);
        load_lsp().iter().for_each(|(hdr, body)| ofib.add_lsa(&hdr, &body.as_slice()));
        //println!("{:#?}", &ofib);

        //println!("{:#?}", ofib.converge(3, 2, 0));
        //for prefixes in ofib.converge(2, 3) {
        //    //println!("prefixes {:#?}", prefixes);
        //}
        panic!();


    }
}
