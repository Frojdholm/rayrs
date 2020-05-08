use std::fs::File;
use std::io::{self, BufRead, BufReader};
use std::path::Path;

#[derive(PartialEq, Eq, Debug)]
enum PlyKeyword {
    Ply,
    Format {
        format: PlyFormat,
        version: String,
    },
    Comment {
        comment: String,
    },
    Element {
        name: String,
        length: usize,
    },
    Property {
        name: String,
        typ: PlyPropertyType,
    },
    ListProperty {
        name: String,
        lentype: PlyPropertyType,
        elemtype: PlyPropertyType,
    },
    EndHeader,
}

impl PlyKeyword {
    /// Converts a single line of the ply header to a token.
    ///
    /// # Examples
    ///
    /// ```
    /// use ply::PlyKeyword;
    /// let line = "element vertex 10";
    /// let keyword = PlyKeyword::from_line(line).unwrap();
    ///
    /// assert_eq!(
    ///     PlyKeyword::Element {
    ///         name: String::from("vertex"),
    ///         length: 10,
    ///     },
    ///     keyword
    /// );
    /// ```
    fn from_line(line: &str) -> io::Result<Self> {
        let mut parts = line.split(" ");
        match parts.next() {
            Some("ply") => Ok(Self::Ply),
            Some("format") => Self::parse_format(parts.collect()),
            Some("comment") => Ok(Self::Comment {
                comment: String::from(line),
            }),
            Some("element") => Self::parse_element(parts.collect()),
            Some("property") => Self::parse_property(parts.collect()),
            Some("end_header") => Ok(Self::EndHeader),
            Some(p) => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("unknown ply keyword: {}", p),
                ))
            }
            None => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "unexpected empty line",
                ))
            }
        }
    }

    fn parse_format(parts: Vec<&str>) -> io::Result<Self> {
        if parts.len() != 2 {
            Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid format specifier",
            ))
        } else if parts[1] != "1.0" {
            Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid version: {}, valid versions: 1.0", parts[1]),
            ))
        } else {
            let format = PlyFormat::from_string(parts[0])?;
            Ok(Self::Format {
                format,
                version: String::from(parts[1]),
            })
        }
    }

    fn parse_element(parts: Vec<&str>) -> io::Result<Self> {
        if parts.len() != 2 {
            Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid element",
            ))
        } else {
            let length = match parts[1].parse::<usize>() {
                Ok(n) => n,
                Err(e) => return Err(io::Error::new(io::ErrorKind::InvalidData, e)),
            };
            Ok(Self::Element {
                name: String::from(parts[0]),
                length,
            })
        }
    }

    fn parse_property(parts: Vec<&str>) -> io::Result<Self> {
        if parts.len() == 2 {
            let typ = PlyPropertyType::from_string(parts[0])?;
            Ok(Self::Property {
                name: String::from(parts[1]),
                typ,
            })
        } else if parts.len() == 4 && parts[0] == "list" {
            let lentype = PlyPropertyType::from_string(parts[1])?;
            let elemtype = PlyPropertyType::from_string(parts[2])?;
            Ok(Self::ListProperty {
                name: String::from(parts[3]),
                lentype,
                elemtype,
            })
        } else {
            Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid property",
            ))
        }
    }
}

#[derive(PartialEq, Eq, Debug)]
enum PlyFormat {
    Ascii,
    BinaryBigEndian,
    BinaryLittleEndian,
}

impl PlyFormat {
    fn from_string(s: &str) -> io::Result<Self> {
        match s {
            "ascii" => Ok(Self::Ascii),
            "binary_big_endian" => Ok(Self::BinaryBigEndian),
            "binary_little_endian" => Ok(Self::BinaryLittleEndian),
            s => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid format: {}", s),
            )),
        }
    }
}

#[derive(PartialEq, Eq, Debug)]
enum PlyPropertyType {
    Char,
    Uchar,
    Short,
    Ushort,
    Int,
    Uint,
    Float,
    Double,
}

impl PlyPropertyType {
    fn from_string(s: &str) -> io::Result<PlyPropertyType> {
        match s {
            "char" => Ok(Self::Char),
            "uchar" => Ok(Self::Uchar),
            "short" => Ok(Self::Short),
            "ushort" => Ok(Self::Ushort),
            "int" => Ok(Self::Int),
            "uint" => Ok(Self::Uint),
            "float" => Ok(Self::Float),
            "double" => Ok(Self::Double),
            s => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid property type: {}", s),
            )),
        }
    }
}

struct PlyHeader {
    version: String,
    format: PlyFormat,
    elements: Vec<PlyKeyword>,
    comments: Vec<String>,
}

impl PlyHeader {
    fn new(version: String, format: PlyFormat) -> PlyHeader {
        PlyHeader {
            version,
            format,
            elements: Vec::new(),
            comments: Vec::new(),
        }
    }

    fn add_comment(&mut self, comment: String) {
        self.comments.push(comment);
    }
    fn add_element(&mut self, name: String, length: usize) {}
    fn add_property(&mut self, name: String, typ: PlyPropertyType) {}
    fn add_list_property(
        &mut self,
        name: String,
        lentype: PlyPropertyType,
        elemtype: PlyPropertyType,
    ) {
    }
}
enum PlyHeaderParser {
    Start,
    Format,
    StartElement(PlyHeader),
    NewElement(PlyHeader),
    InElement(PlyHeader),
    End(PlyHeader),
}

impl PlyHeaderParser {
    fn new() -> PlyHeaderParser {
        PlyHeaderParser::Start
    }

    fn handle_input(self, inp: PlyKeyword) -> io::Result<PlyHeaderParser> {
        match self {
            Self::Start => match inp {
                PlyKeyword::Ply => Ok(Self::Format),
                _ => Err(io::Error::new(
                    io::ErrorKind::Other,
                    "expected 'ply' identifier",
                )),
            },
            Self::Format => match inp {
                PlyKeyword::Format { version, format } => {
                    Ok(Self::StartElement(PlyHeader::new(version, format)))
                }
                _ => Err(io::Error::new(
                    io::ErrorKind::Other,
                    "expected format specification",
                )),
            },
            Self::StartElement(mut header) => match inp {
                PlyKeyword::Comment { comment } => {
                    header.add_comment(comment);
                    Ok(Self::StartElement(header))
                }
                PlyKeyword::Element { name, length } => {
                    header.add_element(name, length);
                    Ok(Self::NewElement(header))
                }
                _ => Err(io::Error::new(
                    io::ErrorKind::Other,
                    "expected 'element' keyword",
                )),
            },
            Self::NewElement(mut header) => match inp {
                PlyKeyword::Comment { comment } => {
                    header.add_comment(comment);
                    Ok(Self::NewElement(header))
                }
                PlyKeyword::Property { name, typ } => {
                    header.add_property(name, typ);
                    Ok(Self::InElement(header))
                }
                PlyKeyword::ListProperty {
                    name,
                    lentype,
                    elemtype,
                } => {
                    header.add_list_property(name, lentype, elemtype);
                    Ok(Self::InElement(header))
                }
                _ => Err(io::Error::new(
                    io::ErrorKind::Other,
                    "expected 'property' keyword",
                )),
            },
            Self::InElement(mut header) => match inp {
                PlyKeyword::Comment { comment } => {
                    header.comments.push(comment);
                    Ok(Self::InElement(header))
                }
                PlyKeyword::Element { name, length } => {
                    header.add_element(name, length);
                    Ok(Self::NewElement(header))
                }
                PlyKeyword::Property { name, typ } => {
                    header.add_property(name, typ);
                    Ok(Self::InElement(header))
                }
                PlyKeyword::ListProperty {
                    name,
                    lentype,
                    elemtype,
                } => {
                    header.add_list_property(name, lentype, elemtype);
                    Ok(Self::InElement(header))
                }
                PlyKeyword::EndHeader => Ok(Self::End(header)),
                _ => Err(io::Error::new(
                    io::ErrorKind::Other,
                    "expected properties or new element",
                )),
            },
            Self::End(_) => panic!("Parser in end state cannot accept more input."),
        }
    }
}

pub struct Ply;

impl Ply {
    pub fn load<T: AsRef<Path>>(path: T) -> io::Result<Ply> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        let mut machine = PlyHeaderParser::new();

        for line in reader.lines() {
            let line = line?;
            let keyword = PlyKeyword::from_line(&line)?;
            machine = machine.handle_input(keyword)?;
            match machine {
                PlyHeaderParser::End(_) => break,
                _ => (),
            }
        }

        let header = match machine {
            PlyHeaderParser::End(header) => header,
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "unexpected EOF",
                ))
            }
        };

        Ok(Ply)
    }
}
