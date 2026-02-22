use std::collections::HashMap;

use super::types::ParamStore;

/// Parse a `.param` file. Each non-comment line should be `NAME,VALUE`.
/// Lines starting with `#` are comments.
pub fn parse_param_file(contents: &str) -> Result<HashMap<String, f32>, String> {
    let mut result = HashMap::new();
    for (line_num, line) in contents.lines().enumerate() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let parts: Vec<&str> = trimmed.splitn(2, ',').collect();
        if parts.len() != 2 {
            return Err(format!("line {}: expected NAME,VALUE", line_num + 1));
        }
        let name = parts[0].trim();
        let value: f32 = parts[1]
            .trim()
            .parse()
            .map_err(|_| format!("line {}: invalid value '{}'", line_num + 1, parts[1].trim()))?;
        result.insert(name.to_string(), value);
    }
    Ok(result)
}

/// Format a `ParamStore` as a `.param` file. Parameters sorted alphabetically.
pub fn format_param_file(store: &ParamStore) -> String {
    let mut names: Vec<&String> = store.params.keys().collect();
    names.sort();
    let mut output = String::new();
    for name in names {
        if let Some(param) = store.params.get(name) {
            output.push_str(&format!("{},{}\n", param.name, param.value));
        }
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::types::{Param, ParamType};

    #[test]
    fn parse_simple() {
        let contents = "BATT_CAPACITY,5000\nBATT_MONITOR,4\n";
        let result = parse_param_file(contents).unwrap();
        assert_eq!(result.len(), 2);
        assert_eq!(result["BATT_CAPACITY"], 5000.0);
        assert_eq!(result["BATT_MONITOR"], 4.0);
    }

    #[test]
    fn parse_with_comments_and_blanks() {
        let contents = "# This is a comment\n\nBATT_CAPACITY,5000\n# Another comment\nBATT_MONITOR,4\n";
        let result = parse_param_file(contents).unwrap();
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn parse_float_values() {
        let contents = "ATC_ACCEL_P_MAX,110000.5\nATC_RAT_PIT_P,0.135\n";
        let result = parse_param_file(contents).unwrap();
        assert!((result["ATC_ACCEL_P_MAX"] - 110000.5).abs() < 0.01);
        assert!((result["ATC_RAT_PIT_P"] - 0.135).abs() < 0.001);
    }

    #[test]
    fn parse_invalid_value() {
        let contents = "BATT_CAPACITY,notanumber\n";
        let result = parse_param_file(contents);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("invalid value"));
    }

    #[test]
    fn parse_missing_comma() {
        let contents = "BATT_CAPACITY\n";
        let result = parse_param_file(contents);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("expected NAME,VALUE"));
    }

    #[test]
    fn format_roundtrip() {
        let mut store = ParamStore::default();
        store.params.insert(
            "BATT_MONITOR".to_string(),
            Param { name: "BATT_MONITOR".to_string(), value: 4.0, param_type: ParamType::Int32, index: 1 },
        );
        store.params.insert(
            "ATC_RAT_PIT_P".to_string(),
            Param { name: "ATC_RAT_PIT_P".to_string(), value: 0.135, param_type: ParamType::Real32, index: 0 },
        );

        let formatted = format_param_file(&store);
        let parsed = parse_param_file(&formatted).unwrap();
        assert_eq!(parsed.len(), 2);
        assert!((parsed["BATT_MONITOR"] - 4.0).abs() < 0.001);
        assert!((parsed["ATC_RAT_PIT_P"] - 0.135).abs() < 0.001);
    }

    #[test]
    fn format_alphabetical_order() {
        let mut store = ParamStore::default();
        store.params.insert(
            "ZEBRA".to_string(),
            Param { name: "ZEBRA".to_string(), value: 1.0, param_type: ParamType::Real32, index: 0 },
        );
        store.params.insert(
            "ALPHA".to_string(),
            Param { name: "ALPHA".to_string(), value: 2.0, param_type: ParamType::Real32, index: 1 },
        );

        let formatted = format_param_file(&store);
        let lines: Vec<&str> = formatted.lines().collect();
        assert!(lines[0].starts_with("ALPHA"));
        assert!(lines[1].starts_with("ZEBRA"));
    }

    #[test]
    fn parse_empty() {
        let result = parse_param_file("").unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn parse_only_comments() {
        let result = parse_param_file("# comment\n# another\n").unwrap();
        assert!(result.is_empty());
    }
}
