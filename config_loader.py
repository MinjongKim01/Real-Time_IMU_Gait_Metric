"""Configuration loader for IMU gait analysis system."""

import yaml
import os
from typing import Any, Dict, Optional
from pathlib import Path


class ConfigLoader:
    """Load and manage YAML configuration with dot notation access."""

    _instance: Optional['ConfigLoader'] = None

    def __new__(cls):
        """Singleton pattern to ensure single config instance."""
        if cls._instance is None:
            cls._instance = super(ConfigLoader, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize configuration loader.

        Args:
            config_path: Path to YAML config file. If None, looks for config.yaml
                        in current directory.
        """
        if self._initialized:
            return

        if config_path is None:
            # Look for config.yaml in current directory
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')

        self.config_path = config_path
        self._config = self._load_config()
        self._initialized = True

    def _load_config(self) -> Dict[str, Any]:
        """
        Load configuration from YAML file.

        Returns:
            Dictionary containing configuration

        Raises:
            FileNotFoundError: If config file doesn't exist
            yaml.YAMLError: If config file is invalid
        """
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")

        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            return config if config is not None else {}
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML config file: {e}")

    def get(self, key_path: str, default: Any = None) -> Any:
        """
        Get configuration value using dot notation.

        Args:
            key_path: Dot-separated path (e.g., 'sensors.sampling_rate')
            default: Default value if key doesn't exist

        Returns:
            Configuration value or default

        Example:
            >>> config = ConfigLoader()
            >>> config.get('sensors.sampling_rate')
            60
            >>> config.get('sensors.mapping.D4:22:CD:00:8D:01')
            'left'
        """
        keys = key_path.split('.')
        value = self._config

        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default

        return value

    def get_section(self, section: str) -> Dict[str, Any]:
        """
        Get entire configuration section.

        Args:
            section: Section name (e.g., 'sensors', 'analysis')

        Returns:
            Dictionary containing section configuration
        """
        return self.get(section, {})

    def reload(self) -> None:
        """Reload configuration from file."""
        self._config = self._load_config()

    def set(self, key_path: str, value: Any) -> None:
        """
        Set configuration value at runtime (not persisted to file).

        Args:
            key_path: Dot-separated path
            value: Value to set
        """
        keys = key_path.split('.')
        config = self._config

        for key in keys[:-1]:
            if key not in config:
                config[key] = {}
            config = config[key]

        config[keys[-1]] = value

    @property
    def all(self) -> Dict[str, Any]:
        """Get entire configuration dictionary."""
        return self._config.copy()


# Global config instance for easy access
_global_config: Optional[ConfigLoader] = None


def get_config(config_path: Optional[str] = None) -> ConfigLoader:
    """
    Get global configuration instance.

    Args:
        config_path: Optional path to config file (only used on first call)

    Returns:
        ConfigLoader instance
    """
    global _global_config
    if _global_config is None:
        _global_config = ConfigLoader(config_path)
    return _global_config
